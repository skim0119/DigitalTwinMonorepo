export type Pose = [number, string, string, string, number, number, number, number, number, number, number];
export type Control = [number, string, number, string, number, number | null, number | null];
export type Action = 'flag' | 'exclude' | 'interpolate';
export type Interval = { body: string; start: number; end: number; action: Action; group?: string };
export type Review = { include: boolean; intervals: Interval[] };
export type Point = { pose: Pose; interpolated?: boolean; gap?: boolean; spread?: number; count?: number };
export type Table = { mode: string; unit: string; duration: number; times: number[]; values: number[][]; interpolation: string };
export type Run = { id: string; batch: string; sequence?: string; trial: number; label: string; table: Table; status: string; error: string | null; simulated: boolean; has_video?: boolean; created: string };
export const active = (run: Run) => ['queued', 'running', 'stopping'].includes(run.status);
export const defaultReview: Review = { include: true, intervals: [] };
export const maxGap = 0.25;
const unit = (q: number[]) => { const norm = Math.hypot(...q); return q.map(v => v / norm); };
const dot = (a: number[], b: number[]) => a.reduce((sum, x, i) => sum + x * b[i], 0);

function slerp(a: number[], b: number[], fraction: number) {
  a = unit(a);
  b = unit(b);
  let cosine = dot(a, b);
  if (cosine < 0) { b = b.map(v => -v); cosine = -cosine; }
  if (cosine > 0.9995) return unit(a.map((x, i) => x + (b[i] - x) * fraction));
  const angle = Math.acos(Math.min(1, cosine));
  const sin = Math.sin(angle);
  return a.map((x, i) => (x * Math.sin((1 - fraction) * angle) + b[i] * Math.sin(fraction * angle)) / sin);
}

function between(a: Pose, b: Pose, time: number): Pose {
  const fraction = (time - a[0]) / (b[0] - a[0]);
  const positions = a.slice(4, 7).map((x, i) => Number(x) + (Number(b[i + 4]) - Number(x)) * fraction);
  return [time, '', a[2], a[3], ...positions, ...slerp(a.slice(7) as number[], b.slice(7) as number[], fraction)] as Pose;
}

export function reviewed(poses: Pose[], intervals: Interval[], body: string): Point[] {
  const marks = intervals.filter(mark => mark.body === body && mark.action !== 'flag');
  const raw = poses.filter(p => p[3] === body).sort((a, b) => a[0] - b[0]);
  const kept = raw.filter(p => !marks.some(mark => mark.start <= p[0] && p[0] <= mark.end));
  const output: Point[] = [];
  for (const pose of kept) {
    const previous = output.at(-1)?.pose;
    if (!previous) { output.push({ pose }); continue; }
    const crossed = marks.filter(mark => mark.start < pose[0] && mark.end > previous[0]);
    const delta = pose[0] - previous[0];
    const sameFrame = pose[2] === previous[2];
    const repair = crossed.length > 0 && crossed.every(mark => mark.action === 'interpolate') && delta <= maxGap && sameFrame;
    if (repair) {
      const timestamps = new Set<number>();
      for (let t = previous[0] + 0.01; t < pose[0]; t += 0.01) timestamps.add(t);
      for (const mark of crossed) timestamps.add((Math.max(mark.start, previous[0]) + Math.min(mark.end, pose[0])) / 2);
      for (const time of [...timestamps].sort((a, b) => a - b)) {
        output.push({ pose: between(previous, pose, time), interpolated: true });
      }
    }
    output.push({ pose, gap: !sameFrame || delta > maxGap || (crossed.length > 0 && !repair) });
  }
  return output;
}

function at(points: Point[], time: number): Pose | undefined {
  let lo = 0, hi = points.length;
  while (lo < hi) {
    const middle = (lo + hi) >> 1;
    if (points[middle].pose[0] < time) lo = middle + 1;
    else hi = middle;
  }
  const before = points[lo - 1], after = points[lo];
  if (after?.pose[0] === time) return after.pose;
  if (!before || !after || after.gap) return;
  return between(before.pose, after.pose, time);
}

export function average(series: Point[][], duration: number, frame: string): Point[] {
  const result: Point[] = [];
  for (let tick = 0; tick <= Math.floor(duration * 50); tick++) {
    const time = tick / 50;
    const poses = series.map(points => at(points, time)).filter((p): p is Pose => !!p && p[2] === frame);
    if (!poses.length) continue;
    const position = [4, 5, 6].map(i => poses.reduce((sum, p) => sum + Number(p[i]), 0) / poses.length);
    const anchor = unit(poses[0].slice(7) as number[]);
    const sum = [0, 0, 0, 0];
    for (const pose of poses) {
      const q = unit(pose.slice(7) as number[]);
      const sign = dot(q, anchor) < 0 ? -1 : 1;
      q.forEach((value, i) => { sum[i] += value * sign; });
    }
    const spread = Math.sqrt(poses.reduce((total, p) => total + position.reduce(
      (distance, value, i) => distance + (Number(p[i + 4]) - value) ** 2, 0), 0) / poses.length);
    const pose = [time, '', frame, poses[0][3], ...position, ...unit(sum)] as Pose;
    result.push({ pose, spread, count: poses.length, gap: !result.length || time - result.at(-1)!.pose[0] > 0.021 });
  }
  return result;
}

export function exportCsv(points: Point[], kind: 'reviewed' | 'mean') {
  const escape = (value: unknown) => '"' + String(value ?? '').replaceAll('"', '""') + '"';
  const header = 'elapsed_s,ros_stamp_ns,frame,body,x_m,y_m,z_m,qx,qy,qz,qw,gap_before,' +
    (kind === 'mean' ? 'spread_m,contributors' : 'interpolated');
  return [header, ...points.map(p => [...p.pose, !!p.gap, ...(kind === 'mean' ?
    [p.spread, p.count] : [!!p.interpolated])].map(escape).join(','))].join('\n') + '\n';
}

export const sampleKey = (pose: Pose) => JSON.stringify([pose[3], pose[0]]);

export function sampleDeletions(poses: Pose[], selection: Set<string>, group: string): Interval[] {
  const bodies = new Map<string, Pose[]>();
  for (const pose of poses) {
    if (!bodies.has(pose[3])) bodies.set(pose[3], []);
    bodies.get(pose[3])!.push(pose);
  }
  const intervals: Interval[] = [];
  for (const [body, rows] of bodies) {
    let interval: Interval | undefined;
    for (const pose of rows.sort((a, b) => a[0] - b[0])) {
      if (!selection.has(sampleKey(pose))) {
        interval = undefined;
      } else if (interval) {
        interval.end = pose[0];
      } else {
        interval = { body, start: pose[0], end: pose[0], action: 'exclude', group };
        intervals.push(interval);
      }
    }
  }
  return intervals;
}
