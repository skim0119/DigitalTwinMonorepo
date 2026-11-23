import { readBendingCsv } from './bending';
import type { Control, Pose, Run } from './review';

export type LocalResult = {
  id: string; kind: 'bundle' | 'bending'; label: string; path: string;
  created: string; files: string[];
};

export const localFile = (id: string, file: string) => `/local-results/${id}/${file}`;

async function get(path: string) {
  const response = await fetch(path);
  if (!response.ok) throw new Error(path === '/local-results'
    ? 'Could not read local results. Check the output folder and refresh.'
    : 'Could not open the recording. Refresh the list and try again.');
  return response;
}

export async function listLocalResults(): Promise<LocalResult[]> {
  return (await get('/local-results')).json();
}

function rows(text: string) {
  return text.replace(/^\uFEFF/, '').trimEnd().split(/\r?\n/).slice(1)
    .filter(Boolean).map(line => [...line.matchAll(/(?:"((?:[^"]|"")*)"|([^",]*))(,|$)/g)]
      .filter(match => match[0] !== '')
      .map(match => match[1]?.replace(/""/g, '"') ?? match[2])
      .concat(line.endsWith(',') ? [''] : []));
}

export async function openLocalResult(entry: LocalResult) {
  const file = async (name: string) => (await get(localFile(entry.id, name))).text();
  if (entry.kind === 'bending') {
    return readBendingCsv(await file('bending.csv'), entry.label, entry.id, entry.created);
  }
  const [metadata, poseCsv, controlCsv] = await Promise.all([
    file('run.json'),
    entry.files.includes('poses.csv') ? file('poses.csv') : '',
    entry.files.includes('controls.csv') ? file('controls.csv') : '',
  ]);
  const run = JSON.parse(metadata) as Run;
  const poses = rows(poseCsv).flatMap<Pose>(fields => {
    const [time, stamp, frame, body, ...values] = fields;
    const numbers = values.map(Number);
    return fields.length === 11 && time.trim() && Number.isFinite(Number(time)) &&
      frame && body && values.every(value => value.trim()) && numbers.every(Number.isFinite) &&
      Math.hypot(...numbers.slice(3)) ? [[Number(time), stamp, frame, body, ...numbers] as Pose] : [];
  });
  const controls = rows(controlCsv).flatMap<Control>(fields => {
    const [time, tendon, setpoint, unit, load, position, speed] = fields;
    if (fields.length !== 7 || !/^tendon_[1-6]$/.test(tendon) ||
        ![time, setpoint, load].every(v => v.trim() && Number.isFinite(Number(v)))) return [];
    return [[Number(time), tendon, Number(setpoint), unit, Number(load),
      position && Number.isFinite(Number(position)) ? Number(position) : null,
      speed && Number.isFinite(Number(speed)) ? Number(speed) : null]];
  });
  if (!poses.length && !controls.length) throw new Error('This recording contains no usable pose or control samples.');
  return [{ run: { ...run, id: entry.id, batch: 'local:' + run.batch, has_video: false,
    created: run.created || entry.created }, poses, controls }];
}
