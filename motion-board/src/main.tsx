import { useEffect, useMemo, useState } from 'react';
import { createRoot } from 'react-dom/client';
import { average, defaultReview, exportCsv, reviewed, sampleDeletions } from './review';
import type { Action, Control, Point, Pose, Review, Run } from './review';
import { readBendingCsv } from './bending';
import { listLocalResults, localFile, openLocalResult } from './local-results';
import type { LocalResult } from './local-results';
import { Trajectory } from './trajectory';
import type { Trace } from './trajectory';
import './style.css';

const palette = ['#2456a6', '#087c77', '#a44726', '#7848a3', '#87590c', '#4a7332'];
const seconds = (value: number) => value.toFixed(2);
const emptyPoses: Pose[] = [];
const emptyControls: Control[] = [];

function download(name: string, contents: string) {
  const url = URL.createObjectURL(new Blob([contents], { type: 'text/csv' }));
  const anchor = document.createElement('a');
  anchor.href = url;
  anchor.download = name;
  anchor.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

function App() {
  const [runs, setRuns] = useState<Run[]>([]);
  const [localResults, setLocalResults] = useState<LocalResult[]>([]);
  const [localLoading, setLocalLoading] = useState(true);
  const [localError, setLocalError] = useState('');
  const [localQuery, setLocalQuery] = useState('');
  const [openedResults, setOpenedResults] = useState<Record<string, LocalResult>>({});
  const [openingResult, setOpeningResult] = useState('');
  const localMatches = localResults.filter(result =>
    `${result.label} ${result.path}`.toLowerCase().includes(localQuery.trim().toLowerCase()));
  const [selected, setSelected] = useState<string[]>([]);
  const [focus, setFocus] = useState('');
  const [poses, setPoses] = useState<Record<string, Pose[]>>({});
  const [controls, setControls] = useState<Record<string, Control[]>>({});
  const [reviews, setReviews] = useState<Record<string, Review>>({});
  const [error, setError] = useState('');
  const [body, setBody] = useState('');
  const [tendon, setTendon] = useState('tendon_1');
  const [cursor, setCursor] = useState(0);
  const [showMean, setShowMean] = useState(false);
  const [start, setStart] = useState<number>();
  const [end, setEnd] = useState<number>();
  const [action, setAction] = useState<Action>('flag');
  const [rawView, setRawView] = useState(false);
  const [picking, setPicking] = useState(false);
  const [samples, setSamples] = useState(new Set<string>());

  const report = (reason: unknown) => setError(String(reason instanceof Error ? reason.message : reason));
  async function refreshLocal() {
    setLocalLoading(true); setLocalError('');
    try { setLocalResults(await listLocalResults()); }
    catch (reason) { setLocalError(reason instanceof Error ? reason.message : String(reason)); }
    finally { setLocalLoading(false); }
  }
  useEffect(() => { void refreshLocal(); }, []);
  const current = runs.find(run => run.id === focus);
  const openResult = current && (openedResults[current.id] ?? openedResults[current.batch]);
  const focusPoses = poses[focus] ?? emptyPoses;
  const focusControls = controls[focus] ?? emptyControls;
  const duration = Math.max(.01, current?.table.duration ?? 1);
  const last = focusPoses.at(-1)?.[0] ?? focusControls.at(-1)?.[0] ?? 0;
  const timelineDuration = Math.max(duration, last, ...runs.filter(run => selected.includes(run.id))
    .map(run => Math.max(run.table.duration, poses[run.id]?.at(-1)?.[0] ?? 0)));
  const time = cursor;
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
      return [{ key: 'mean:' + name, color: '#152441', mean: true, points: average(series, duration, frame) }];
    });
  }, [showMean, batchKey, body, bodies, poses, reviews, duration]);
  const traces = useMemo(() => [...traceData, ...mean], [traceData, mean]);
  const meanPoints = mean.flatMap(trace => trace.points);
  const meanAtTime = meanPoints.filter(p => Math.abs(p.pose[0] - time) <= .011);
  const cursorPose = body ? focusPoses.reduce<Pose | undefined>((closest, candidate) =>
    candidate[3] === body && (!closest || Math.abs(candidate[0] - time) < Math.abs(closest[0] - time)) ? candidate : closest,
  undefined) : undefined;
  const canReview = !!current;
  const canPick = canReview && selected.includes(focus) && !rawView && focusPoses.length > 0;
  const lastDeletion = [...review.intervals].reverse().find(mark => mark.group)?.group;
  const marks = review.intervals.filter(mark => !body || mark.body === body);

  function choose(run: Run) {
    setPicking(false); setSamples(new Set());
    setFocus(run.id); setBody(''); setCursor(0); setStart(undefined); setEnd(undefined);
    setSelected(ids => ids.includes(run.id) ? ids : [...ids, run.id]);
  }
  function showRecords(records: Awaited<ReturnType<typeof openLocalResult>>) {
    const ids = records.map(record => record.run.id);
    setRuns(previous => [...records.map(record => record.run), ...previous.filter(run => !ids.includes(run.id))]);
    setPoses(previous => ({ ...previous, ...Object.fromEntries(records.map(record => [record.run.id, record.poses])) }));
    setControls(previous => ({ ...previous, ...Object.fromEntries(records.map(record => [record.run.id, record.controls])) }));
    choose(records[0].run);
    setSelected(ids);
    const ranges = new Map<string, [number, number]>();
    for (const [, name, command] of records[0].controls) {
      const [low, high] = ranges.get(name) ?? [command, command];
      ranges.set(name, [Math.min(low, command), Math.max(high, command)]);
    }
    const [mostActive] = [...ranges].sort(([, a], [, b]) => (b[1] - b[0]) - (a[1] - a[0]));
    setTendon(mostActive?.[0] ?? 'tendon_1');
  }
  async function openRecording(entry: LocalResult) {
    setOpeningResult(entry.id); setLocalError('');
    try {
      showRecords(await openLocalResult(entry));
      setOpenedResults(previous => ({ ...previous, [entry.id]: entry }));
    } catch (reason) { setLocalError(reason instanceof Error ? reason.message : String(reason)); }
    finally { setOpeningResult(''); }
  }
  async function importBending(file: File) {
    try {
      const batch = crypto.randomUUID().replaceAll('-', '');
      showRecords(readBendingCsv(await file.text(), file.name, batch, new Date(file.lastModified).toISOString()));
      setError('');
    } catch (error) { report(error); }
  }

  async function saveReview(next: Review) {
    setReviews(current => ({ ...current, [focus]: next }));
    return true;
  }

  async function deleteSamples() {
    if (!canPick || !samples.size) return;
    const intervals = sampleDeletions(focusPoses, samples, crypto.randomUUID());
    if (await saveReview({ ...review, intervals: [...review.intervals, ...intervals] })) setSamples(new Set());
  }

  function undoDeletion(group: string) {
    void saveReview({ ...review, intervals: review.intervals.filter(mark => mark.group !== group) });
  }

  return <>
    <header className="top"><h1>Motion Board</h1><p>Experiment recordings</p></header>
    {error && <p className="error" role="alert">{error}</p>}
    <div className="workspace">
      <aside><h2>Recordings</h2>
        <div className="local-library">
          <div className="local-search">
            <input type="search" aria-label="Filter local recordings" placeholder="Filter recordings"
              value={localQuery} onChange={event => setLocalQuery(event.target.value)} />
            <button type="button" onClick={() => void refreshLocal()} disabled={localLoading}>Refresh</button>
          </div>
          {localError && <p className="local-message" role="alert">{localError}</p>}
          <p className="local-count" role="status">{localLoading ? 'Scanning local output…' :
            `${localMatches.length} ${localMatches.length === 1 ? 'recording' : 'recordings'}`}</p>
          {!localLoading && !localMatches.length && !localError && <p className="local-message">
            {localQuery ? 'No recordings match your filter.' : 'No matching recordings in the local output folder.'}</p>}
          {localMatches.length > 0 && <ul className="local-recordings" aria-label="Local recordings">
            {localMatches.map(entry => <li key={entry.id}>
              <button type="button" aria-current={entry.id === openResult?.id ? 'true' : undefined}
                disabled={!!openingResult} title={entry.path} onClick={() => void openRecording(entry)}>
                <span>{entry.label}</span>
                <small>{entry.kind === 'bundle' ? 'Run bundle' : 'Bending CSV'} · {new Date(entry.created).toLocaleDateString()}</small>
                {openingResult === entry.id && <small role="status">Opening…</small>}
              </button>
            </li>)}
          </ul>}
          <label className="file local-import">Import bending CSV<input type="file" accept=".csv,text/csv" onChange={event => {
            const file = event.currentTarget.files?.[0];
            event.currentTarget.value = '';
            if (file) void importBending(file);
          }} /></label>
        </div>
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
              <small className="run-status" data-status={run.status}>{run.status}</small>
            </div>)}
          </section>;
        })}
      </aside>
      <main>
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
            <p>{current ? `${current.status[0].toUpperCase() + current.status.slice(1)} · ${current.error ||
              `${seconds(current.table.duration)} s${current.simulated ? ' · Simulation' : ''}`}` :
              'Open a recording to review its trajectory and tendon forces.'}</p></div>

          </div>
          <div className="viewport-toolbar">
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
                <button disabled={!samples.size} onClick={() => void deleteSamples()}>Delete selected</button>
                <button disabled={!samples.size} onClick={() => setSamples(new Set())}>Clear selection</button>
              </>}
              {lastDeletion && <button  onClick={() => undoDeletion(lastDeletion)}>Undo last deletion</button>}
            </div>}
          </div>
          <Trajectory traces={traces} cursor={time} scope={selected.join(",") + ":" + body}
            pick={picking && canPick ? focus : ''} selection={samples}
            onPick={keys => setSamples(previous => new Set([...previous, ...keys]))} />
          <div className="keys">{traceData.map(trace => <span key={trace.key}><i style={{ background: trace.color }} />
            Trial {runs.find(run => trace.key.startsWith(run.id + ':'))?.trial} · {trace.key.slice(33)}</span>)}
            {mean.length > 0 && <span><i style={{ background: '#152441' }} />Mean</span>}</div>
          <div className="timeline">
            <div className="timeline-head"><label htmlFor="time">Timeline</label><output>{seconds(time)} / {seconds(timelineDuration)} s</output>
</div>
            <input id="time" type="range" aria-label="Trial time" min={0} max={timelineDuration} step={0.001} value={Math.min(time, timelineDuration)}
              onChange={e => setCursor(Number(e.target.value))} />
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
          {canReview && <section className="review"><h3>Clean recording</h3><p className="review-note">Edits last until this page is closed or refreshed. Download the reviewed CSV to keep them; original files stay unchanged.</p>
            <label className="check"><input type="checkbox" checked={review.include}
              onChange={e => void saveReview({ ...review, include: e.target.checked })} /> Include this trial in the mean</label>
            <div className="tools">
              <button onClick={() => setStart(time)}>Set start {start === undefined ? '' : seconds(start) + ' s'}</button>
              <button onClick={() => setEnd(time)}>Set end {end === undefined ? '' : seconds(end) + ' s'}</button>
              <select aria-label="Review action" value={action} onChange={e => setAction(e.target.value as Action)}>
                <option value="flag">Flag interval</option><option value="exclude">Exclude samples</option><option value="interpolate">Interpolate short gap</option>
              </select>
              <button disabled={!names.length || start === undefined || end === undefined || start >= end || end > duration}
                onClick={() => void saveReview({ ...review, intervals: [...review.intervals,
                  ...names.map(name => ({ body: name, start: start!, end: end!, action }))] })}>Apply to {body || 'all bodies'}</button>
            </div>
            {review.intervals.map((mark, i) => mark.group
              ? review.intervals.findIndex(candidate => candidate.group === mark.group) === i &&
                <div className="annotation" key={mark.group}>
                  <span>Sample deletion · {review.intervals.filter(candidate => candidate.group === mark.group).length} selected segments</span>
                  <button  onClick={() => undoDeletion(mark.group!)}>Undo deletion</button>
                </div>
              : <div className="annotation" key={i}>
              <span>{mark.body} · {seconds(mark.start)}–{seconds(mark.end)} s · {mark.action}</span>
              <button  onClick={() => void saveReview({ ...review, intervals: review.intervals.filter((_, index) => index !== i) })}>Undo</button></div>)}
          </section>}
          {current && <section className="downloads" aria-label="Recording files">
            <h3>Export and original files</h3>
            {focusPoses.length > 0 && <button onClick={() => download(`${focus}_reviewed.csv`,
              exportCsv([...new Set(focusPoses.map(p => p[3]))].flatMap(name => reviewed(focusPoses, review.intervals, name)), 'reviewed'))}>Download reviewed CSV</button>}
            {meanPoints.length > 0 && <button onClick={() => download(`${current.batch}_mean.csv`, exportCsv(meanPoints, 'mean'))}>Download mean CSV</button>}
            {openResult?.files.map(file => <a key={file} href={localFile(openResult.id, file)} download>{file === 'bending.csv' ? 'Original recording CSV' : file}</a>)}
          </section>}

        </section>
      </main>
    </div>
  </>;
}

function ControlPlot({ rows, tendon, time, duration }: { rows: Control[]; tendon: string; time: number; duration: number }) {
  const data = rows.filter(row => row[1] === tendon);
  if (!data.length) return <p className="control-empty">No control or load samples in this trial.</p>;
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
