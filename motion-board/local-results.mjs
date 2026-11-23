import { createReadStream } from 'node:fs';
import { open, readdir, readFile, stat } from 'node:fs/promises';
import { createHash } from 'node:crypto';
import { basename, join, relative } from 'node:path';

const headers = {
  bending: 'cycle,elapsed_s,ros_stamp_ns,frame,body,x_m,y_m,z_m,qx,qy,qz,qw,tendon,target_kgf,load_kgf,run_status,simulated',
  'poses.csv': 'elapsed_s,ros_stamp_ns,frame,body,x_m,y_m,z_m,qx,qy,qz,qw',
  'controls.csv': 'elapsed_s,tendon,setpoint,unit,load_kgf,position_steps,output_steps_s',
};
const terminal = new Set(['complete', 'cancelled', 'failed', 'interrupted']);

async function header(path) {
  const file = await open(path);
  try {
    const buffer = Buffer.alloc(256);
    const { bytesRead } = await file.read(buffer);
    return buffer.toString('utf8', 0, bytesRead).replace(/^\uFEFF/, '').split(/\r?\n/, 1)[0];
  } finally { await file.close(); }
}

export function localResults(root) {
  let entries = new Map();

  async function scan() {
    const found = new Map();
    async function walk(folder) {
      const children = await readdir(folder, { withFileTypes: true });
      const files = new Set(children.filter(child => child.isFile()).map(child => child.name));
      async function add(path, kind, assets, label) {
        const id = createHash('sha256').update(relative(root, path)).digest('hex').slice(0, 32);
        const created = (await stat(path)).mtime.toISOString();
        found.set(id, { id, kind, created, path: relative(root, path), label, assets });
      }

      if (files.has('run.json')) {
        try {
          const metaPath = join(folder, 'run.json');
          const run = JSON.parse(await readFile(metaPath, 'utf8'));
          if (terminal.has(run.status) && typeof run.label === 'string' && Number.isFinite(run.table?.duration)) {
            const assets = { 'run.json': metaPath };
            for (const name of ['poses.csv', 'controls.csv']) {
              if (files.has(name) && await header(join(folder, name)) === headers[name]) {
                assets[name] = join(folder, name);
              }
            }
            if (assets['poses.csv'] || assets['controls.csv']) {
              for (const name of ['actuation.csv', 'webcam.mp4']) {
                if (files.has(name)) assets[name] = join(folder, name);
              }
              await add(folder, 'bundle', assets, run.label);
            }
          }
        } catch (error) {
          if (error.code && error.code !== 'ENOENT') throw error;
        }
      }

      for (const child of children) {
        const path = join(folder, child.name);
        if (child.isDirectory() && !child.name.startsWith('.')) await walk(path);
        if (child.isFile() && child.name.endsWith('.csv') && await header(path) === headers.bending) {
          await add(path, 'bending', { 'bending.csv': path }, basename(path, '.csv'));
        }
      }
    }
    try { await walk(root); }
    catch (error) {
      if (error.code !== 'ENOENT') throw error;
    }
    entries = found;
    return [...found.values()].sort((a, b) => b.created.localeCompare(a.created) || a.path.localeCompare(b.path))
      .map(({ assets, ...entry }) => ({ ...entry, files: Object.keys(assets) }));
  }

  return async (path, res) => {
    if (path === '/local-results') {
      try {
        const results = await scan();
        res.writeHead(200, { 'Content-Type': 'application/json', 'Cache-Control': 'no-store' });
        return res.end(JSON.stringify(results));
      } catch {
        return res.writeHead(500).end('Could not read the local output directory.');
      }
    }
    const [, , id, name] = path.split('/');
    const assets = entries.get(id)?.assets;
    const file = assets && Object.hasOwn(assets, name) ? assets[name] : undefined;
    if (!file || !/^\/local-results\/[a-f0-9]{32}\/[\w.]+$/.test(path)) return res.writeHead(404).end();
    const stream = createReadStream(file);
    stream.on('open', () => {
      res.writeHead(200, { 'Content-Type': name.endsWith('.csv') ? 'text/csv' :
        name.endsWith('.mp4') ? 'video/mp4' : 'application/json', 'Cache-Control': 'no-store' });
      stream.pipe(res);
    });
    stream.on('error', () => res.headersSent ? res.destroy() : res.writeHead(404).end());
    res.on('close', () => stream.destroy());
  };
}
