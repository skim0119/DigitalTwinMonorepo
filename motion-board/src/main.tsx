import { useEffect, useMemo, useState } from 'react';
import type { FormEvent } from 'react';
import { createRoot } from 'react-dom/client';
import { active, average, defaultReview, exportCsv, reviewed, sampleDeletions } from './review';
import type { Action, Control, Point, Pose, Review, Run, Table } from './review';
import { readBendingCsv } from './bending';
import { Trajectory } from './trajectory';
import type { Trace } from './trajectory';
import './style.css';

const palette = ['#71e1c7', '#8db9ff', '#f8a782', '#d0a1ec', '#edcf84', '#9bd88b'];
const seconds = (value: number) => value.toFixed(2);
const urlFor = (id: string, name: string) => `/api/runs/${id}/files/${name}?download=true`;
const emptyPoses: Pose[] = [];
const emptyControls: Control[] = [];

async function api<T>(path: string, options?: RequestInit): Promise<T> {
  const response = await fetch('/api' + path, options);
  if (response.ok) return response.json();
  const body = await response.json().catch(() => ({}));
  throw new Error(typeof body.detail === 'string' ? body.detail : `Experiment API returned ${response.status}.`);
}

function download(name: string, contents: string) {
  const url = URL.createObjectURL(new Blob([contents], { type: 'text/csv' }));
  const anchor = document.createElement('a');
  anchor.href = url;
  anchor.download = name;
  anchor.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

function App() {
  const [liveRuns, setRuns] = useState<Run[]>([]);
  const [importedRuns, setImportedRuns] = useState<Run[]>([]);
  const runs = [...importedRuns, ...liveRuns];
  const [selected, setSelected] = useState<string[]>([]);
  const [focus, setFocus] = useState('');
  const [poses, setPoses] = useState<Record<string, Pose[]>>({});
  const [controls, setControls] = useState<Record<string, Control[]>>({});
  const [reviews, setReviews] = useState<Record<string, Review>>({});
  const [simulated, setSimulated] = useState<boolean>();
  const [online, setOnline] = useState(false);
  const [error, setError] = useState('');
  const [pending, setPending] = useState(false);
  const [form, showForm] = useState(false);
  const [mode, setMode] = useState('force');
  const [unit, setUnit] = useState('N');
  const [raw, setRaw] = useState('');
  const [table, setTable] = useState<Table>();
  const [label, setLabel] = useState('');
  const [repeats, setRepeats] = useState(1);
  const [confirmed, confirm] = useState(false);
  const [body, setBody] = useState('');
  const [tendon, setTendon] = useState('tendon_1');
  const [cursor, setCursor] = useState(0);
  const [follow, setFollow] = useState(true);
  const [showMean, setShowMean] = useState(false);
  const [start, setStart] = useState<number>();
  const [end, setEnd] = useState<number>();
  const [action, setAction] = useState<Action>('flag');
  const [rawView, setRawView] = useState(false);
  const [picking, setPicking] = useState(false);
  const [samples, setSamples] = useState(new Set<string>());

  const report = (reason: unknown) => setError(String(reason instanceof Error ? reason.message : reason));
  const refresh = () => api<Run[]>('/runs').then(setRuns);
  useEffect(() => {
    let stopped = false;
    let timer: ReturnType<typeof setTimeout>;
    let stateVersion = "";
    let socket: WebSocket;
    function connect() {
      socket = new WebSocket(`${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}/api/events`);
      socket.onopen = () => {
        if (stopped) return;
        setOnline(true);
        setError('');
        void refresh().catch(report);
        void api<{ simulated: boolean }>('/health').then(m => setSimulated(m.simulated)).catch(report);
      };
      socket.onmessage = event => {
        if (stopped) return;
        const data = JSON.parse(event.data) as {
          states: Record<string, [string, string | null]>;
          frames: [number, string, 'poses' | 'controls', Pose[] | Control[]][];
        };
        setRuns(current => current.map(run => {
          const state = data.states[run.id];
          return state ? { ...run, status: state[0], error: state[1] } : run;
        }));
        if (data.frames.length) {
          const incomingPoses: Record<string, Pose[]> = {}, incomingControls: Record<string, Control[]> = {};
          for (const [, id, kind, rows] of data.frames) {
            if (kind === 'poses') (incomingPoses[id] ??= []).push(...rows as Pose[]);
            else (incomingControls[id] ??= []).push(...rows as Control[]);
          }
          if (Object.keys(incomingPoses).length) setPoses(current => append(current, incomingPoses));
          if (Object.keys(incomingControls).length) setControls(current => append(current, incomingControls));
        }
        const version = JSON.stringify(data.states);
        if (version !== stateVersion) { stateVersion = version; void refresh().catch(report); }
      };
      socket.onclose = () => {
        if (stopped) return;
        setOnline(false);
        timer = setTimeout(connect, 1500);
      };
    }
    connect();
    return () => { stopped = true; clearTimeout(timer); socket.close(); };
  }, []);

  const archiveKey = liveRuns.filter(run => selected.includes(run.id) && !active(run)).map(run => run.id).join(',');
  useEffect(() => {
    if (!archiveKey || !online) return;
    let dead = false;
    for (const id of archiveKey.split(',')) {
      Promise.all([
        api<Pose[]>(`/runs/${id}/samples?kind=poses`), api<Control[]>(`/runs/${id}/samples?kind=controls`),
        api<Review>(`/runs/${id}/review`),
      ]).then(([p, c, review]) => {
        if (dead) return;
        setPoses(state => ({ ...state, [id]: p }));
        setControls(state => ({ ...state, [id]: c }));
        setReviews(state => ({ ...state, [id]: review }));
      }).catch(error => { if (!dead) report(error); });
    }
    return () => { dead = true; };
  }, [archiveKey, online]);

  const current = runs.find(run => run.id === focus);
  const imported = importedRuns.some(run => run.id === focus);
  const busy = liveRuns.find(run => run.status === 'running' || run.status === 'stopping') ?? liveRuns.find(active);
  const focusPoses = poses[focus] ?? emptyPoses;
  const focusControls = controls[focus] ?? emptyControls;
  const duration = Math.max(.01, current?.table.duration ?? 1);
  const last = focusPoses.at(-1)?.[0] ?? focusControls.at(-1)?.[0] ?? 0;
  const timelineDuration = Math.max(duration, last, ...runs.filter(run => selected.includes(run.id))
    .map(run => Math.max(run.table.duration, poses[run.id]?.at(-1)?.[0] ?? 0)));
  const time = follow && current && active(current) ? last : cursor;
  const bodies = useMemo(() => [...new Set(selected.flatMap(id => (poses[id] ?? []).map(p => p[3])))].sort(), [poses, selected]);
  const review = reviews[focus] ?? defaultReview;
  const selectedRuns = runs.filter(run => selected.includes(run.id));
  const focusBodies = [...new Set(focusPoses.map(p => p[3]))];
  const names = body ? focusBodies.filter(name => name === body) : focusBodies;
  const traceData = useMemo(() => {
    const traces: Trace[] = [];
    for (const id of selected) for (const [index, name] of (body ? [body] : bodies).entries()) {
      const points = reviewed(poses[id] ?? [], rawView ? [] : (reviews[id]?.intervals ?? []), name);
      if (points.length) traces.push({ key: id + ':' + name, color: palette[(selected.indexOf(id) + index) % palette.length], points });
    }
    return traces;
  }, [selected, body, bodies, poses, rawView, reviews]);
  const batchRuns = selectedRuns.filter(run => current && (run.sequence ?? run.batch) === (current.sequence ?? current.batch)
    && run.simulated === current.simulated && run.status === 'complete'
    && (reviews[run.id] ?? defaultReview).include);
  const batchKey = batchRuns.map(r => r.id).join(',');
  const mean = useMemo(() => {
    if (!showMean || !batchKey.includes(',')) return [];
    const ids = batchKey.split(',');
    return (body ? [body] : bodies).flatMap(name => {
      const series = ids.map(id => reviewed(poses[id] ?? [], reviews[id]?.intervals ?? [], name)).filter(points => points.length);
      const frame = series[0]?.[0].pose[2];
      if (!frame) return [];
      return [{ key: 'mean:' + name, color: '#fff1cf', mean: true, points: average(series, duration, frame) }];
    });
  }, [showMean, batchKey, body, bodies, poses, reviews, duration]);
  const traces = useMemo(() => [...traceData, ...mean], [traceData, mean]);
  const meanPoints = mean.flatMap(trace => trace.points);
  const meanAtTime = meanPoints.filter(p => Math.abs(p.pose[0] - time) <= .011);
  const cursorPose = body ? focusPoses.reduce<Pose | undefined>((closest, candidate) =>
    candidate[3] === body && (!closest || Math.abs(candidate[0] - time) < Math.abs(closest[0] - time)) ? candidate : closest,
  undefined) : undefined;
  const canReview = !!current && !imported && !active(current);
  const canPick = canReview && selected.includes(focus) && !rawView && focusPoses.length > 0;
  const lastDeletion = [...review.intervals].reverse().find(mark => mark.group)?.group;
  const marks = review.intervals.filter(mark => !body || mark.body === body);

  function choose(run: Run) {
    setPicking(false); setSamples(new Set());
    setFocus(run.id); setCursor(0); setFollow(true); setStart(undefined); setEnd(undefined);
    setSelected(ids => ids.includes(run.id) ? ids : [...ids, run.id]);
  }
  async function importBending(file: File) {
    try {
      const batch = crypto.randomUUID().replaceAll('-', '');
      const records = readBendingCsv(await file.text(), file.name, batch, new Date(file.lastModified).toISOString());
      setImportedRuns(previous => [...records.map(record => record.run), ...previous]);
      setPoses(previous => ({ ...previous, ...Object.fromEntries(records.map(record => [record.run.id, record.poses])) }));
      setControls(previous => ({ ...previous, ...Object.fromEntries(records.map(record => [record.run.id, record.controls])) }));
      choose(records[0].run);
      setSelected(records.map(record => record.run.id));
      setBody('');
      setTendon(records[0].controls[0]?.[1] ?? 'tendon_1');
      setError('');
    } catch (error) { report(error); }
  }

  async function submit(event: FormEvent) {
    event.preventDefault();
    setError(''); setPending(true);
    const query = new URLSearchParams({ mode, unit });
    try {
      if (!table) {
        setTable(await api<Table>('/validate?' + query, { method: 'POST', body: raw }));
      } else {
        query.set('label', label || 'Actuation sequence'); query.set('repeats', String(repeats));
        query.set('reference_ready', String(confirmed));
        const started = await api<Run[]>('/runs?' + query, { method: 'POST', body: raw });
        setRuns(previous => [...started, ...previous.filter(run => !started.some(next => next.id === run.id))]);
        setSelected([started[0].id]);
        choose(started[0]);
        showForm(false); confirm(false);
      }
    } catch (error) { report(error); }
    finally { setPending(false); }
  }
  async function repeat(run: Run) {
    setError(''); setPending(true);
    try {
      const response = await fetch(`/api/runs/${run.id}/files/actuation.csv`);
      if (!response.ok) throw new Error('Could not reload the actuation CSV.');
      setRaw(await response.text());
      setTable(run.table); setMode(run.table.mode); setUnit(run.table.unit); setLabel(run.label);
      setRepeats(1); confirm(false); showForm(true);
      window.scrollTo(0, 0);
    } catch (error) { report(error); }
    finally { setPending(false); }
  }
  async function saveReview(next: Review) {
    setError(''); setPending(true);
    try {
      const saved = await api<Review>(`/runs/${focus}/review`, {
        method: 'PUT', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(next),
      });
      setReviews(current => ({ ...current, [focus]: saved }));
      return true;
    } catch (error) { report(error); }
    finally { setPending(false); }
  }

  async function deleteSamples() {
    if (!canPick || pending || !samples.size) return;
    const intervals = sampleDeletions(focusPoses, samples, crypto.randomUUID());
    if (await saveReview({ ...review, intervals: [...review.intervals, ...intervals] })) setSamples(new Set());
  }

  function undoDeletion(group: string) {
    if (!pending) void saveReview({ ...review, intervals: review.intervals.filter(mark => mark.group !== group) });
  }

  return <>
    <header className="top"><h1>Motion Board</h1>
      <div className="top-actions"><span className={online ? 'online' : 'offline'}>{online ? simulated ? 'Simulation' : 'Pi connected' : 'Disconnected'}</span>
      <button className="primary" onClick={() => showForm(!form)}>{form ? 'Close setup' : 'New experiment'}</button></div></header>
    {error && <p className="error" role="alert">{error}</p>}
    <div className="workspace">
      <aside><h2>Runs <small>{runs.length}</small></h2>
        <label className="file">Import bending CSV<input type="file" accept=".csv,text/csv" onChange={e => {
          const file = e.currentTarget.files?.[0];
          e.currentTarget.value = '';
          if (file) void importBending(file);
        }} /></label>
        {runs.length === 0 && <p>{online ? 'Upload an actuation table to record your first trial.' : 'Waiting for Experiment API.'}</p>}
        {[...new Set(runs.map(run => run.batch))].map(batch => {
          const children = runs.filter(run => run.batch === batch).sort((a, b) => a.trial - b.trial);
          const first = children[0];
          return <section key={batch} className="batch">
            <h3>{first.label}</h3><p>{first.table.mode} · {first.table.unit} · {new Date(first.created).toLocaleString()}</p>
            {children.map(run => <div className={focus === run.id ? 'run focused' : 'run'} key={run.id}>
              <input type="checkbox" checked={selected.includes(run.id)} aria-label={`Overlay ${run.label} trial ${run.trial}`}
                onChange={e => {
                  if (e.target.checked) {
                    if (!focus) choose(run);
                    else setSelected(ids => [...ids, run.id]);
                  } else {
                    setSelected(ids => ids.filter(id => id !== run.id));
                    if (focus === run.id) {
                      const next = runs.find(candidate => candidate.id !== run.id && selected.includes(candidate.id));
                      if (next) choose(next);
                      else { setFocus(''); setSamples(new Set()); setPicking(false); setCursor(0); }
                    }
                  }
                }} />
              <button onClick={() => choose(run)}>Trial {run.trial}</button>
              <small>{run.status}</small>
            </div>)}
          </section>;
        })}
      </aside>
      <main>
        {form && <form className="setup" onSubmit={submit}>
          <h2>Actuation table</h2>
          <div className="form-fields">
            <label>Experiment name<input value={label} maxLength={100} onChange={e => setLabel(e.target.value)} placeholder="Actuation sequence" /></label>
            <label>Control mode<select value={mode} onChange={e => { setMode(e.target.value); setUnit(e.target.value === 'force' ? 'N' : 'mm'); setTable(undefined); }}>
              <option value="force">Force</option><option value="displacement">Tendon displacement</option></select></label>
            <label>Units<select value={unit} onChange={e => { setUnit(e.target.value); setTable(undefined); }}>
              {mode === 'force' ? <><option>N</option><option>kgf</option></> : <option>mm</option>}</select></label>
            <label>Trials<input type="number" min={1} max={30} value={repeats} onChange={e => setRepeats(Number(e.target.value))} /></label>
            <label className="file">CSV file<input type="file" accept=".csv,text/csv" onChange={async e => {
              setTable(undefined); setRaw('');
              const file = e.target.files?.[0];
              if (file && file.size > 256 * 1024) return report('Choose a CSV smaller than 256 KiB.');
              if (file) { try { setRaw(await file.text()); } catch (error) { report(error); } }
            }} /></label>
          </div>
          {table && <details open><summary>{table.times.length} rows · {table.duration} s · {table.interpolation} between rows</summary>
            <div className="table-scroll"><table><thead><tr><th>Time (s)</th>{table.values[0].map((_, i) => <th key={i}>Tendon {i + 1}</th>)}</tr></thead>
              <tbody>{table.values.map((row, i) => <tr key={table.times[i]}><td>{table.times[i]}</td>{row.map((value, j) => <td key={j}>{value}</td>)}</tr>)}</tbody></table></div>
          </details>}
          {table && simulated === false && <label className="check"><input type="checkbox" checked={confirmed} onChange={e => confirm(e.target.checked)} /> Arm is at the calibrated zero and preload</label>}
          <button className="primary" disabled={pending || !online || !raw || (!!table && (!!busy || (simulated === false && !confirmed)))}>
            {table ? 'Start trials' : 'Validate CSV'}</button>
        </form>}
        <section className="workbench" onKeyDown={event => {
          if (!picking || !canPick || event.target instanceof HTMLInputElement || event.target instanceof HTMLSelectElement) return;
          if (event.key === 'Delete' || event.key === 'Backspace') {
            event.preventDefault();
            void deleteSamples();
          } else if ((event.metaKey || event.ctrlKey) && event.key.toLowerCase() === 'z' && lastDeletion) {
            event.preventDefault();
            undoDeletion(lastDeletion);
          }
        }}>
          <div className="workhead"><div><h2>{current ? `${current.label} · Trial ${current.trial}` : 'Pose trajectory'}</h2>
            <p>{current?.error ?? (current ? `${current.status} · ${seconds(current.table.duration)} s${current.simulated ? ' · Simulation' : ''}` : 'Select trials to compare their recorded motion.')}</p></div>
            {busy && <button className="stop" onClick={() => { void api(`/runs/${busy.id}/stop`, { method: 'POST' }).then(refresh).catch(report); }}>Stop batch</button>}
          </div>
          <div className="tools">
            <label>Rigid body<select value={body} onChange={e => { setSamples(new Set()); setBody(e.target.value); }}>
              <option value="">All bodies</option>{bodies.map(name => <option key={name}>{name}</option>)}</select></label>
            <label className="check"><input type="checkbox" checked={showMean} onChange={e => setShowMean(e.target.checked)} /> Mean of included repeats</label>
            <label className="check"><input type="checkbox" checked={rawView} onChange={e => { setPicking(false); setSamples(new Set()); setRawView(e.target.checked); }} /> Show raw poses</label>
          </div>
          {canReview && <div className="tools sample-tools">
            <button aria-pressed={picking && canPick} disabled={!canPick} onClick={() => {
              setPicking(!picking); setSamples(new Set());
            }}>{picking && canPick ? 'Done selecting' : 'Select samples'}</button>
            {picking && canPick && <>
              <span aria-live="polite">{samples.size ? `${samples.size} selected` : 'Drag a box over the focused trial'}</span>
              <button disabled={!samples.size || pending} onClick={() => void deleteSamples()}>Delete selected</button>
              <button disabled={!samples.size} onClick={() => setSamples(new Set())}>Clear selection</button>
            </>}
            {lastDeletion && <button disabled={pending} onClick={() => undoDeletion(lastDeletion)}>Undo last deletion</button>}
          </div>}
          <Trajectory traces={traces} cursor={time} scope={selected.join(",") + ":" + body}
            pick={picking && canPick ? focus : ''} selection={samples}
            onPick={keys => setSamples(previous => new Set([...previous, ...keys]))} />
          <div className="keys">{traceData.map(trace => <span key={trace.key}><i style={{ background: trace.color }} />
            Trial {runs.find(run => trace.key.startsWith(run.id + ':'))?.trial} · {trace.key.slice(33)}</span>)}
            {mean.length > 0 && <span><i style={{ background: '#fff1cf' }} />Mean</span>}</div>
          <div className="timeline">
            <div className="timeline-head"><label htmlFor="time">Timeline</label><output>{seconds(time)} / {seconds(timelineDuration)} s</output>
              {current && active(current) && <label className="check"><input type="checkbox" checked={follow} onChange={e => setFollow(e.target.checked)} /> Follow live</label>}</div>
            <input id="time" type="range" aria-label="Trial time" min={0} max={timelineDuration} step={0.001} value={Math.min(time, timelineDuration)}
              onChange={e => { setCursor(Number(e.target.value)); setFollow(false); }} />
            {marks.length > 0 && <div className="mark-track" aria-label="Annotated intervals">{marks.map((mark, index) =>
              <span key={index} title={mark.action} className={mark.action} style={{ left: 100 * mark.start / timelineDuration + '%', width: `max(2px, ${100 * (mark.end - mark.start) / timelineDuration}%)` }} />)}</div>}
            {cursorPose && Math.abs(cursorPose[0] - time) < .1 && <p className="stat">
              Raw {cursorPose[2]} position · {cursorPose.slice(4, 7).map(value => Number(value).toFixed(4)).join(', ')} m
            </p>}
            {meanAtTime.length > 0 && <p className="stat">{meanAtTime.map(p =>
              `${p.pose[3]} · ${p.count} contributing trials · ${((p.spread ?? 0) * 1000).toFixed(2)} mm RMS spread`).join(' | ')}</p>}
            {current && <><label>Tendon<select value={tendon} onChange={e => setTendon(e.target.value)}>
              {Array.from({ length: 6 }, (_, i) => <option key={i} value={`tendon_${i + 1}`}>Tendon {i + 1}</option>)}</select></label>
              <ControlPlot rows={focusControls} tendon={tendon} time={time} duration={timelineDuration} /></>}
          </div>
          {canReview && <section className="review"><h3>Review intervals</h3>
            <label className="check"><input type="checkbox" checked={review.include} disabled={pending}
              onChange={e => void saveReview({ ...review, include: e.target.checked })} /> Include this trial in the mean</label>
            <div className="tools">
              <button onClick={() => setStart(time)}>Set start {start === undefined ? '' : seconds(start) + ' s'}</button>
              <button onClick={() => setEnd(time)}>Set end {end === undefined ? '' : seconds(end) + ' s'}</button>
              <select aria-label="Review action" value={action} onChange={e => setAction(e.target.value as Action)}>
                <option value="flag">Flag interval</option><option value="exclude">Exclude samples</option><option value="interpolate">Interpolate short gap</option>
              </select>
              <button disabled={pending || !names.length || start === undefined || end === undefined || start >= end || end > duration}
                onClick={() => void saveReview({ ...review, intervals: [...review.intervals,
                  ...names.map(name => ({ body: name, start: start!, end: end!, action }))] })}>Apply to {body || 'all bodies'}</button>
            </div>
            {review.intervals.map((mark, i) => mark.group
              ? review.intervals.findIndex(candidate => candidate.group === mark.group) === i &&
                <div className="annotation" key={mark.group}>
                  <span>Sample deletion · {review.intervals.filter(candidate => candidate.group === mark.group).length} selected segments</span>
                  <button disabled={pending} onClick={() => undoDeletion(mark.group!)}>Undo deletion</button>
                </div>
              : <div className="annotation" key={i}>
              <span>{mark.body} · {seconds(mark.start)}–{seconds(mark.end)} s · {mark.action}</span>
              <button disabled={pending} onClick={() => void saveReview({ ...review, intervals: review.intervals.filter((_, index) => index !== i) })}>Undo</button></div>)}
          </section>}
          {current && !imported && <div className="downloads">
            <a href={urlFor(focus, 'actuation.csv')}>Actuation CSV</a>
            {focusPoses.length > 0 && <a href={urlFor(focus, 'poses.csv')}>Raw poses CSV</a>}
            {focusControls.length > 0 && <a href={urlFor(focus, 'controls.csv')}>Controls CSV</a>}
            {current.has_video && <a href={urlFor(focus, 'webcam.mp4')}>Webcam video (MP4)</a>}
            <a href={urlFor(focus, 'run.json')}>Trial metadata</a>
            {(review.intervals.length > 0 || !review.include) && <a href={urlFor(focus, 'review.json')}>Review annotations</a>}
            {canReview && <button disabled={!online || pending} onClick={() => void repeat(current)}>Reuse actuation table</button>}
            {canReview && focusPoses.length > 0 && <button onClick={() => download(`${focus}_reviewed.csv`,
              exportCsv([...new Set(focusPoses.map(p => p[3]))].flatMap(name => reviewed(focusPoses, review.intervals, name)), 'reviewed'))}>Reviewed CSV</button>}
            {meanPoints.length > 0 && <button onClick={() => download(`${current.batch}_mean.csv`, exportCsv(meanPoints, 'mean'))}>Mean CSV</button>}
          </div>}
        </section>
      </main>
    </div>
  </>;
}

function append<T>(current: Record<string, T[]>, incoming: Record<string, T[]>) {
  const next = { ...current };
  for (const [id, rows] of Object.entries(incoming)) next[id] = [...(current[id] ?? []), ...rows].slice(-4000);
  return next;
}

function ControlPlot({ rows, tendon, time, duration }: { rows: Control[]; tendon: string; time: number; duration: number }) {
  const data = rows.filter(row => row[1] === tendon);
  if (!data.length) return <p>Awaiting control and load data.</p>;
  const makeLine = (index: 2 | 4) => {
    let low = 0, high = 0;
    for (const row of data) { low = Math.min(low, row[index]); high = Math.max(high, row[index]); }
    const range = high - low || 1;
    return data.map(row => `${(row[0] / duration * 600).toFixed(2)},${(48 - (row[index] - low) / range * 40).toFixed(2)}`).join(' ');
  };
  const nearest = data.reduce((a, b) => Math.abs(a[0] - time) <= Math.abs(b[0] - time) ? a : b);
  return <div className="control-plot">
    <svg aria-label="Command and measured load over time, each independently scaled" role="img" viewBox="0 0 600 55" preserveAspectRatio="none">
      <polyline points={makeLine(2)} className="command-line" /><polyline points={makeLine(4)} className="load-line" />
      <line x1={time / duration * 600} x2={time / duration * 600} y1="0" y2="55" />
    </svg>
    <div><span>Command {nearest[2].toFixed(3)} {nearest[3]}</span><span>Load {nearest[4].toFixed(3)} kgf</span></div>
  </div>;
}

createRoot(document.getElementById('root')!).render(<App />);
