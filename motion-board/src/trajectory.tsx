import { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { sampleKey } from './review';
import type { Point } from './review';
import type { PointerEvent } from 'react';

export type Trace = { key: string; color: string; points: Point[]; mean?: boolean };
const xyz = (p: Point) => new THREE.Vector3(p.pose[4], p.pose[5], p.pose[6]);

type Box = { x: number; y: number; left: number; top: number; width: number; height: number };

export function Trajectory({ traces, cursor, scope, pick, selection, onPick }: {
  traces: Trace[]; cursor: number; scope: string; pick: string;
  selection: Set<string>; onPick: (keys: string[]) => void;
}) {
  const mount = useRef<HTMLDivElement>(null);
  const view = useRef<{ scene: THREE.Scene; camera: THREE.PerspectiveCamera; controls: OrbitControls; data: THREE.Group; grid: THREE.GridHelper } | null>(null);
  const [error, setError] = useState(false);
  const [box, setBox] = useState<Box>();
  const markers = useRef<{ trace: Trace; axis: THREE.AxesHelper }[]>([]);
  const fit = useRef({ scope: "", extent: 0, moved: false });
  useEffect(() => {
    const target = mount.current!;
    let renderer: THREE.WebGLRenderer;
    try { renderer = new THREE.WebGLRenderer({ antialias: true }); }
    catch { setError(true); return; }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setClearColor('#ffffff');
    target.appendChild(renderer.domElement);
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(42, 1, 0.0001, 200);
    camera.up.set(0, 0, 1);
    camera.position.set(0.5, -0.7, 0.5);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.addEventListener("start", () => { fit.current.moved = true; });
    controls.minDistance = 0.005;
    controls.maxDistance = 100;
    const grid = new THREE.GridHelper(2, 20, '#98aac4', '#d5deeb');
    grid.rotation.x = Math.PI / 2;
    scene.add(grid);
    const data = new THREE.Group();
    scene.add(data);
    view.current = { scene, camera, controls, data, grid };
    const resize = new ResizeObserver(() => {
      if (!target.clientHeight) return;
      camera.aspect = target.clientWidth / target.clientHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(target.clientWidth, target.clientHeight);
    });
    resize.observe(target);
    renderer.setAnimationLoop(() => renderer.render(scene, camera));
    return () => {
      resize.disconnect();
      controls.dispose();
      renderer.setAnimationLoop(null);
      grid.dispose();
      dispose(data);
      renderer.dispose();
      renderer.domElement.remove();
      view.current = null;
    };
  }, []);

  useEffect(() => {
    const state = view.current;
    if (!state) return;
    dispose(state.data);
    markers.current = [];
    const bounds = new THREE.Box3();
    traces.forEach(trace => {
      const solid: THREE.Vector3[] = [], dotted: THREE.Vector3[] = [], dots: THREE.Vector3[] = [];
      trace.points.forEach((point, i) => {
        const position = xyz(point);
        bounds.expandByPoint(position);
        if (trace.mean) dots.push(position);
        if (!i || point.gap) return;
        const previous = trace.points[i - 1];
        const segment = point.interpolated || previous.interpolated ? dotted : solid;
        segment.push(xyz(previous), position);
      });
      const add = (coordinates: THREE.Vector3[], dash: boolean) => {
        if (!coordinates.length) return;
        const geometry = new THREE.BufferGeometry().setFromPoints(coordinates);
        const material = dash ? new THREE.LineDashedMaterial({ color: trace.color, dashSize: .004, gapSize: .003 })
          : new THREE.LineBasicMaterial({ color: trace.color });
        const line = new THREE.LineSegments(geometry, material);
        if (dash) line.computeLineDistances();
        state.data.add(line);
      };
      add(solid, false);
      add(dotted, true);
      if (dots.length) state.data.add(new THREE.Points(new THREE.BufferGeometry().setFromPoints(dots),
        new THREE.PointsMaterial({ color: trace.color, size: 4, sizeAttenuation: false })));
      const axis = new THREE.AxesHelper(.012);
      axis.setColors("#ae3e34", "#277047", "#365da7");
      markers.current.push({ trace, axis });
      state.data.add(axis);
    });
    const extent = Math.max(bounds.getSize(new THREE.Vector3()).length(), .03);
    if (!bounds.isEmpty() && (fit.current.scope !== scope ||
        (!fit.current.moved && extent > fit.current.extent * 1.7))) {
      const center = bounds.getCenter(new THREE.Vector3());
      state.controls.target.copy(center);
      state.camera.position.copy(center).add(new THREE.Vector3(.76, -1.19, .76).multiplyScalar(extent));
      state.grid.position.set(center.x, center.y, bounds.min.z - extent * .12);
      state.grid.scale.setScalar(Math.max(extent, .05) * 2);
      state.controls.update();
      fit.current = { scope, extent, moved: false };
    }
    const chosen = traces.filter(trace => pick && trace.key.startsWith(pick + ':')).flatMap(trace =>
      trace.points.filter(point => !point.interpolated && selection.has(sampleKey(point.pose))).map(xyz));
    if (chosen.length) state.data.add(new THREE.Points(new THREE.BufferGeometry().setFromPoints(chosen),
      new THREE.PointsMaterial({ color: '#a96303', size: 7, sizeAttenuation: false, depthTest: false })));
    if (bounds.isEmpty()) fit.current = { scope: "", extent: 0, moved: false };
  }, [traces, scope, pick, selection]);

  useEffect(() => {
    for (const { trace, axis } of markers.current) {
      let lo = 0, hi = trace.points.length;
      while (lo < hi) {
        const middle = (lo + hi) >> 1;
        if (trace.points[middle].pose[0] < cursor) lo = middle + 1;
        else hi = middle;
      }
      const candidates = [trace.points[lo - 1], trace.points[lo]].filter(Boolean);
      const point = candidates.sort((a, b) => Math.abs(a.pose[0] - cursor) - Math.abs(b.pose[0] - cursor))[0];
      axis.visible = !!point && Math.abs(point.pose[0] - cursor) < 0.1;
      if (axis.visible) {
        axis.position.copy(xyz(point));
        axis.quaternion.set(point.pose[7], point.pose[8], point.pose[9], point.pose[10]).normalize();
      }
    }
  }, [cursor, traces]);

  function reset() {
    const state = view.current;
    if (!state || !traces.some(t => t.points.length)) return;
    const box = new THREE.Box3().setFromObject(state.data);
    const center = box.getCenter(new THREE.Vector3());
    const scale = Math.max(box.getSize(new THREE.Vector3()).length(), .03);
    state.controls.target.copy(center);
    state.camera.position.copy(center).add(new THREE.Vector3(.76, -1.19, .76).multiplyScalar(scale));
    state.controls.update();
    fit.current.moved = true;
  }

  function drag(event: PointerEvent<HTMLDivElement>, done: boolean) {
    if (!box || !view.current) return;
    const area = event.currentTarget.getBoundingClientRect();
    const x = event.clientX - area.left, y = event.clientY - area.top;
    const rect = { ...box, left: Math.min(x, box.x), top: Math.min(y, box.y),
      width: Math.abs(x - box.x), height: Math.abs(y - box.y) };
    if (!done) { setBox(rect); return; }
    const padding = rect.width < 4 && rect.height < 4 ? 5 : 0;
    const camera = view.current.camera;
    camera.updateMatrixWorld();
    const keys = traces.filter(trace => trace.key.startsWith(pick + ':')).flatMap(trace =>
      trace.points.filter(point => {
        if (point.interpolated) return false;
        const p = xyz(point).project(camera);
        const px = (1 + p.x) * area.width / 2, py = (1 - p.y) * area.height / 2;
        return Math.abs(p.z) <= 1 && px >= rect.left - padding && px <= rect.left + rect.width + padding
          && py >= rect.top - padding && py <= rect.top + rect.height + padding;
      }).map(point => sampleKey(point.pose)));
    onPick(keys);
    setBox(undefined);
  }
  return <div className="trajectory">
    <div ref={mount} className="viewport" aria-label="Rotatable 3D pose trajectories" />
    {error ? <p className="viewport-note">3D rendering is unavailable. Timeline and CSV downloads still work.</p>
      : !traces.some(t => t.points.length) && <p className="viewport-note">Select a recorded trial to inspect its trajectory.</p>}
    {!error && <button className="view-reset" onClick={reset}>Fit view</button>}
    {pick && !error && <div className="sample-pick" tabIndex={0} aria-label="Drag to select samples from the focused trial"
      onPointerDown={event => {
        if (event.button !== 0) return;
        event.currentTarget.focus();
        event.currentTarget.setPointerCapture(event.pointerId);
        const rect = event.currentTarget.getBoundingClientRect();
        const x = event.clientX - rect.left, y = event.clientY - rect.top;
        setBox({ x, y, left: x, top: y, width: 0, height: 0 });
      }}
      onPointerMove={event => drag(event, false)} onPointerUp={event => drag(event, true)}
      onPointerCancel={() => setBox(undefined)} onLostPointerCapture={() => setBox(undefined)}>
      {box && <div className="sample-box" style={{ left: box.left, top: box.top, width: box.width, height: box.height }} />}
    </div>}
    <span className="axis-key"><b>X</b><b>Y</b><b>Z</b> <span>meters</span></span>
  </div>;
}

function dispose(group: THREE.Group) {
  for (const object of [...group.children]) {
    if (object instanceof THREE.Line || object instanceof THREE.Points) {
      object.geometry.dispose();
      (object.material as THREE.Material).dispose();
    }
    group.remove(object);
  }
}
