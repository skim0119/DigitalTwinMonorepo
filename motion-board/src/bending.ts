import type { Control, Pose, Run } from './review';

const header = 'cycle,elapsed_s,ros_stamp_ns,frame,body,x_m,y_m,z_m,qx,qy,qz,qw,tendon,target_kgf,load_kgf,run_status,simulated';

export function readBendingCsv(text: string, name: string, batch: string, created: string) {
  const [first, ...lines] = text.replace(/^\uFEFF/, '').trimEnd().split(/\r?\n/);
  if (first !== header) throw new Error('Choose an Exp 1 or Exp 2 bending collection CSV.');
  const cycles = new Map<number, { run: Run; poses: Pose[]; controls: Control[]; controlTimes: Map<string, number> }>();
  for (const [index, line] of lines.entries()) {
    // The exporter quotes fields containing commas or quotes. Pose frame IDs are single-line.
    const fields = [...line.matchAll(/(?:"((?:[^"]|"")*)"|([^",]*))(,|$)/g)]
      .map(match => match[1]?.replace(/""/g, '"') ?? match[2]);
    if (fields.length === 18 && fields.at(-1) === '') fields.pop();
    const [cycle, elapsed, stamp, frame, body, ...rest] = fields;
    const pose = rest.slice(0, 7).map(value => value === '' ? NaN : Number(value));
    const [tendon, target, load, status, simulated] = rest.slice(7);
    const trial = Number(cycle), time = Number(elapsed);
    if (fields.length !== 17 || !Number.isInteger(trial) || trial < 1 || trial > 30 ||
        elapsed === '' || !Number.isFinite(time) || time < 0 || !/^\d+$/.test(stamp) ||
        !frame || !body.trim() || !pose.every(Number.isFinite) ||
        !Math.hypot(...pose.slice(3)) || !/^tendon_[1-6]$/.test(tendon) ||
        target === '' || !Number.isFinite(Number(target)) ||
        (load !== '' && !Number.isFinite(Number(load))) ||
        !['complete', 'cancelled', 'failed', 'interrupted'].includes(status) ||
        !['True', 'False'].includes(simulated)) {
      throw new Error(`Invalid bending data on row ${index + 2}.`);
    }
    let entry = cycles.get(trial);
    if (!entry) {
      const id = batch.slice(0, 30) + trial.toString().padStart(2, '0');
      const run: Run = {
        id, batch, trial, label: name, created, simulated: simulated === 'True', status, error: null,
        table: { mode: 'force', unit: 'kgf', duration: 0, times: [], values: [], interpolation: 'hold' },
      };
      entry = { run, poses: [], controls: [], controlTimes: new Map() };
      cycles.set(trial, entry);
    }
    entry.run.table.duration = Math.max(entry.run.table.duration, time);
    const previous = entry.poses.at(-1);
    if (!previous || previous[0] !== time || previous[1] !== stamp || previous[3] !== body) {
      entry.poses.push([time, stamp, frame, body, ...pose] as Pose);
    }
    if (load !== '' && entry.controlTimes.get(tendon) !== time) {
      entry.controlTimes.set(tendon, time);
      entry.controls.push([time, tendon, Number(target), 'kgf', Number(load), null, null]);
    }
  }
  if (!cycles.size) throw new Error('This file contains no recorded poses.');
  return [...cycles.values()].sort((a, b) => a.run.trial - b.run.trial);
}
