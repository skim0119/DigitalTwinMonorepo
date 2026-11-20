import { strict as assert } from 'node:assert';
import { test } from 'node:test';
import { average, exportCsv, reviewed } from './review.ts';
import type { Pose } from './review.ts';

const pose = (time: number, x: number, w = 1): Pose => [time, '1795155618921776823', 'world', 'Tip, "A"', x, 0, 0, 0, 0, 0, w];

test('review masks spikes and retains raw data and quaternion sign equivalence', () => {
  const body = 'Tip, "A"';
  const raw = [pose(0, 0), pose(.04, 99), pose(.08, .08, -1)];
  const excluded = reviewed(raw, [{ body, start: .03, end: .05, action: 'exclude' }], body);
  assert.equal(excluded.length, 2);
  assert.equal(excluded[1].gap, true);
  const fixed = reviewed(raw, [{ body, start: .03, end: .05, action: 'interpolate' }], body);
  assert.ok(fixed.some(p => p.interpolated && Math.abs(p.pose[4] - .04) < .00001 && p.pose[10] === 1));
  assert.ok(fixed.every(p => p.pose[4] < 1));
  assert.equal(raw[1][4], 99);
  assert.match(exportCsv(fixed, 'reviewed'), /"Tip, ""A"""/);
});

test('repeat mean tracks spread, sample count, and gaps', () => {
  const body = 'Tip, "A"';
  const a = reviewed([pose(0, 0), pose(.10, 2), pose(.50, 5)], [], body);
  const b = reviewed([pose(0, 2, -1), pose(.10, 4, -1)], [], body);
  const mean = average([a, b], .5, 'world');
  assert.equal(mean[0].pose[4], 1);
  assert.equal(mean[0].count, 2);
  assert.equal(mean[0].spread, 1);
  assert.ok(Math.abs(mean[0].pose[10]) === 1);
  assert.ok(!mean.some(p => p.pose[0] === .2));
  assert.equal(mean.at(-1)?.count, 1);
});

test('sample deletion groups consecutive selections and preserves unselected frames', async () => {
  const { sampleDeletions, sampleKey } = await import('./review.ts');
  const raw = [0, .02, .04, .06, .08].map(t => pose(t, t));
  const marks = sampleDeletions(raw, new Set([0, 1, 3].map(i => sampleKey(raw[i]))), 'delete-one');
  assert.deepEqual(marks.map(mark => [mark.start, mark.end, mark.group]),
    [[0, .02, 'delete-one'], [.06, .06, 'delete-one']]);
  assert.deepEqual(reviewed(raw, marks, raw[0][3]).map(p => p.pose[0]), [.04, .08]);
  assert.equal(reviewed(raw, marks, raw[0][3])[1].gap, true);
  assert.equal(reviewed(raw, marks.filter(mark => mark.group !== 'delete-one'), raw[0][3]).length, 5);
});
