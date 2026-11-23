"""Shared HTTP client for local experiment scripts."""

import argparse
import json
import math
import re
import shutil
import sys
import time
from pathlib import Path
from tempfile import mkdtemp
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode, urlsplit
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]
ACTIVE = {"queued", "running", "stopping"}
FILES = ("actuation.csv", "run.json", "poses.csv", "controls.csv")


def request_json(url, raw=None):
    request = Request(url, data=raw, headers={"Content-Type": "text/csv"} if raw is not None else {})
    with urlopen(request, timeout=15) as response:
        return json.load(response)


def download(url, destination):
    with urlopen(url, timeout=30) as response:
        pending = destination.with_suffix(".part")
        try:
            with pending.open("wb") as stream:
                shutil.copyfileobj(response, stream)
            pending.replace(destination)
        finally:
            pending.unlink(missing_ok=True)


def _run_trial(args, raw, duration, *, label, folder_prefix, mode, unit, required_body_prefix):
    base_url = args.api_url.rstrip("/")
    output = args.output_dir
    output.mkdir(parents=True, exist_ok=True)
    folder = Path(mkdtemp(prefix=folder_prefix, dir=output))
    (folder / "actuation.csv").write_bytes(raw)
    print(f"Actuation CSV: {folder / 'actuation.csv'}", flush=True)

    query = urlencode({"mode": mode, "unit": unit})
    request_json(f"{base_url}/validate?{query}", raw)
    if args.dry_run:
        print(f"Dry run validated | scheduled actuation {duration:g} s")
        return 0

    options = {"label": label, "reference_ready": "true"}
    if required_body_prefix:
        options["required_body_prefix"] = required_body_prefix
    if args.name:
        options["name"] = args.name
    try:
        (run,) = request_json(f"{base_url}/runs?{query}&{urlencode(options)}", raw)
    except (TimeoutError, URLError) as error:
        if isinstance(error, HTTPError):
            raise
        raise RuntimeError("Submission outcome is unknown. Check GET /runs before resubmitting.") from error

    run_id = run["id"]
    files_url = f"{base_url}/runs/{run_id}/files"
    status_url = f"{files_url}/run.json"
    print(f"Trial {run_id} | scheduled actuation {duration:g} s", flush=True)
    (folder / "run.json").write_text(json.dumps(run, indent=2) + "\n")
    deadline = time.monotonic() + duration + 120
    while run["status"] in ACTIVE:
        if time.monotonic() >= deadline:
            raise RuntimeError(f"Stopped waiting for {run_id}. Check {status_url} for its current status.")
        time.sleep(1)
        updated = request_json(status_url)
        if updated["status"] != run["status"]:
            print(f"Trial {run_id} | {updated['status']}", flush=True)
        run = updated

    files = (*FILES, "webcam.mp4") if run.get("has_video") else FILES
    for name in files:
        try:
            download(f"{files_url}/{name}", folder / name)
        except HTTPError as error:
            if error.code != 404 or run["status"] == "complete" or name in ("actuation.csv", "run.json"):
                raise
            print(f"No {name} was recorded.", file=sys.stderr)

    print(f"Trial {run_id} | {run['status']} | results: {folder}")
    if run.get("error"):
        print(run["error"], file=sys.stderr)
    return 0 if run["status"] == "complete" else 1


def add_api_arguments(parser):
    parser.add_argument("--api-url", default="http://127.0.0.1:5052",
                        help="base URL of the reachable Experiment API")
    parser.add_argument("--dry-run", action="store_true",
                        help="save and validate the actuation CSV without submitting a trial")
    parser.add_argument("--name", help="append a name to the server trial ID")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "output" / "experiment")


def parse_api_arguments(parser, argv=None):
    args = parser.parse_args(argv)
    try:
        url = urlsplit(args.api_url)
    except ValueError as error:
        parser.error(str(error))
    if url.scheme not in ("http", "https") or not url.netloc or url.query or url.fragment:
        parser.error("--api-url must be an HTTP(S) base URL without a query or fragment.")
    if args.name is not None and not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", args.name):
        parser.error("--name must contain 1–80 letters, digits, hyphens or underscores.")
    return args


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("expected a finite positive number")
    return number


def run_trial(args, raw, duration, *, label, folder_prefix, mode, unit, required_body_prefix=None):
    try:
        return _run_trial(
            args, raw, duration, label=label, folder_prefix=folder_prefix, mode=mode,
            unit=unit, required_body_prefix=required_body_prefix,
        )
    except HTTPError as error:
        with error:
            detail = error.read(4096).decode(errors="replace")
        print(f"Experiment API returned HTTP {error.code}: {detail}", file=sys.stderr)
    except (OSError, RuntimeError, ValueError) as error:
        print(error, file=sys.stderr)
    except KeyboardInterrupt:
        print("Stopped waiting locally. Check GET /runs for the trial status.", file=sys.stderr)
        return 130
    return 1
