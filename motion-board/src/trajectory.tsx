import { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import type { Point } from './review';

export type Trace = { key: string; color: string; points: Point[]; mean?: boolean };
const xyz = (p: Point) => new THREE.Vector3(p.pose[4], p.pose[5], p.pose[6]);

export function Trajectory({ traces, cursor, scope }: { traces: Trace[]; cursor: number; scope: string }) {
  const mount = useRef<HTMLDivElement>(null);
  const view = useRef<{ scene: THREE.Scene; camera: THREE.PerspectiveCamera; controls: OrbitControls; data: THREE.Group; grid: THREE.GridHelper } | null>(null);
  const [error, setError] = useState(false);
  const markers = useRef<{ trace: Trace; axis: THREE.AxesHelper }[]>([]);
  const fit = useRef({ scope: "", extent: 0, moved: false });
  useEffect(() => {
    const target = mount.current!;
    let renderer: THREE.WebGLRenderer;
    try { renderer = new THREE.WebGLRenderer({ antialias: true }); }
    catch { setError(true); return; }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setClearColor('#15232a');
    target.appendChild(renderer.domElement);
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(42, 1, 0.0001, 200);
    camera.up.set(0, 0, 1);
    camera.position.set(0.5, -0.7, 0.5);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.addEventListener("start", () => { fit.current.moved = true; });
    controls.minDistance = 0.005;
    controls.maxDistance = 100;
    const grid = new THREE.GridHelper(2, 20, '#496572', '#273d48');
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
      axis.setColors("#ff9c91", "#9fe8a6", "#a1caff");
      markers.current.push({ trace, axis });
      state.data.add(axis);
    });
    const extent = Math.max(bounds.getSize(new THREE.Vector3()).length(), .03);
    if (!bounds.isEmpty() && (fit.current.scope !== scope ||
        (!fit.current.moved && extent > fit.current.extent * 1.7))) {
      const center = bounds.getCenter(new THREE.Vector3());
      state.controls.target.copy(center);
      state.camera.position.copy(center).add(new THREE.Vector3(.9, -1.4, .9).multiplyScalar(extent));
      state.grid.position.set(center.x, center.y, bounds.min.z - extent * .12);
      state.grid.scale.setScalar(Math.max(extent, .05) * 2);
      state.controls.update();
      fit.current = { scope, extent, moved: false };
    }
    if (bounds.isEmpty()) fit.current = { scope: "", extent: 0, moved: false };
  }, [traces, scope]);

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
    state.camera.position.copy(center).add(new THREE.Vector3(.9, -1.4, .9).multiplyScalar(scale));
    state.controls.update();
    fit.current.moved = true;
  }

  return <div className="trajectory">
    <div ref={mount} className="viewport" aria-label="Rotatable 3D pose trajectories" />
    {error ? <p className="viewport-note">3D rendering is unavailable. Timeline and CSV downloads still work.</p>
      : !traces.some(t => t.points.length) && <p className="viewport-note">Select a recorded trial to inspect its trajectory.</p>}
    {!error && <button className="view-reset" onClick={reset}>Fit view</button>}
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
