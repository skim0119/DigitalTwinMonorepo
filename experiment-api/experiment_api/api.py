"""HTTP commands and downloads, with a read-only WebSocket preview."""
import asyncio
import csv
import math
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException, Query, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
from pydantic import BaseModel, Field

from .runner import ACTIVE, VIDEO_FILE, BusyError, write_json
from .table import MAX_CSV, parse_table

FILES = {"actuation.csv", "poses.csv", "controls.csv", "run.json", "review.json", VIDEO_FILE}


class Interval(BaseModel):
    body: str = Field(min_length=1, max_length=200)
    start: float
    end: float
    action: str = Field(pattern="^(flag|exclude|interpolate)$")
    group: str | None = Field(default=None, max_length=64)


class Review(BaseModel):
    include: bool = True
    intervals: list[Interval] = Field(default_factory=list, max_length=1000)


def create_app(experiments):
    @asynccontextmanager
    async def lifespan(_app):
        experiments.start_mocap()
        try:
            yield
        finally:
            await asyncio.to_thread(experiments.close)

    app = FastAPI(title="Experiment API", lifespan=lifespan)

    @app.middleware("http")
    async def reject_browser_requests(request, call_next):
        if request.headers.get("origin"):
            return Response(status_code=403)
        return await call_next(request)

    async def table_from(request, mode, unit):
        raw = bytearray()
        async for chunk in request.stream():
            if len(raw) + len(chunk) > MAX_CSV:
                raise HTTPException(413, "Upload a CSV smaller than 256 KiB.")
            raw.extend(chunk)
        try:
            table = parse_table(raw, mode, unit)
            if not experiments.simulated:
                experiments.hardware.calibration.validate(table)
        except ValueError as error:
            raise HTTPException(422, str(error)) from error
        return table, raw

    def find(run_id):
        try:
            return experiments.get(run_id)
        except KeyError as error:
            raise HTTPException(404, "Trial not found.") from error

    @app.get("/health")
    def health():
        return {"simulated": experiments.simulated, "required_body_supported": True}

    @app.post("/validate")
    async def validate(request: Request, mode: str, unit: str):
        table, _ = await table_from(request, mode, unit)
        return table.response()

    @app.get("/runs")
    def runs():
        return experiments.list()

    @app.post("/runs", status_code=202)
    async def start(request: Request, mode: str, unit: str,
                    repeats: int = Query(1, ge=1, le=30), label: str = Query("Experiment", max_length=100),
                    reference_ready: bool = False, required_body: list[str] = Query(default=[])):
        table, raw = await table_from(request, mode, unit)
        if not experiments.simulated and not reference_ready:
            raise HTTPException(422, "Confirm the arm is at its calibrated zero and preload.")
        try:
            return await asyncio.to_thread(experiments.submit, table, raw, label.strip() or "Experiment",
                                           repeats, tuple(required_body))
        except BusyError as error:
            raise HTTPException(409, str(error)) from error

    @app.post("/runs/{run_id}/stop")
    def stop(run_id: str):
        find(run_id)
        try:
            return experiments.stop(run_id)
        except BusyError as error:
            raise HTTPException(409, str(error)) from error

    @app.get("/runs/{run_id}/review")
    def get_review(run_id: str):
        find(run_id)
        path = experiments.root / run_id / "review.json"
        if path.exists():
            return FileResponse(path, media_type="application/json")
        return Review()

    @app.put("/runs/{run_id}/review")
    def put_review(run_id: str, review: Review):
        run = find(run_id)
        if run["status"] in ACTIVE:
            raise HTTPException(409, "Wait until recording finishes before reviewing it.")
        for interval in review.intervals:
            if not all(map(math.isfinite, [interval.start, interval.end])) or not (
                    0 <= interval.start <= interval.end <= run["table"]["duration"]
                    and (interval.start < interval.end or interval.action == "exclude")):
                raise HTTPException(422, "Review intervals must be inside the trial timeline.")
        with experiments.lock:
            write_json(experiments.root / run_id / "review.json", review.model_dump(exclude_none=True))
        return review.model_dump(exclude_none=True)

    @app.get("/runs/{run_id}/samples")
    def samples(run_id: str, kind: str = Query(pattern="^(poses|controls)$")):
        find(run_id)
        path = experiments.root / run_id / f"{kind}.csv"
        if not path.is_file():
            return []
        with path.open(newline="") as stream:
            rows = csv.reader(stream)
            next(rows, None)
            if kind == "poses":
                return [[float(t), stamp, frame, body, *map(float, values)]
                        for t, stamp, frame, body, *values in rows if len(values) == 7]
            return [[float(t), tendon, float(target), unit, float(load),
                     int(steps) if steps else None, float(speed) if speed else None]
                    for t, tendon, target, unit, load, steps, speed in rows]

    @app.get("/runs/{run_id}/files/{name}")
    def file(run_id: str, name: str, download: bool = False):
        find(run_id)
        if name not in FILES:
            raise HTTPException(404, "Unknown run file.")
        path = experiments.root / run_id / name
        if not path.is_file():
            raise HTTPException(404, "This file is not available for the trial.")
        media = {".csv": "text/csv", ".mp4": "video/mp4", ".json": "application/json"}[path.suffix]
        return FileResponse(path, media_type=media, filename=f"{run_id}_{name}" if download else None)

    @app.websocket("/events")
    async def events(ws: WebSocket):
        if ws.headers.get("origin"):
            await ws.close(code=1008)
            return
        await ws.accept()
        cursor = experiments.sequence
        previous = None
        try:
            while True:
                with experiments.lock:
                    runs = list(experiments.runs.values())
                    states = {run["id"]: [run["status"], run["error"]] for run in runs}
                    frames = [frame for frame in experiments.preview if frame[0] > cursor]
                if frames or states != previous:
                    await ws.send_json({"frames": frames, "states": states})
                    if frames:
                        cursor = frames[-1][0]
                    previous = states
                try:
                    await asyncio.wait_for(ws.receive_text(), 0.12)
                except asyncio.TimeoutError:
                    pass
        except WebSocketDisconnect:
            pass

    return app
