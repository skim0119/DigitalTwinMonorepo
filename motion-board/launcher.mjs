import { createReadStream, existsSync, statSync } from 'node:fs';
import { createServer } from 'node:http';
import { join, extname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { parseArgs } from 'node:util';
import { localResults } from './local-results.mjs';

const { values } = parseArgs({ options: {
  'output-dir': { type: 'string' },
  port: { type: 'string', default: '8082' },
}});
function port(value) {
  const number = Number(value);
  if (!Number.isInteger(number) || number < 1 || number > 65535) throw new Error('Invalid port');
  return number;
}
const localPort = port(values.port);
const results = localResults(values['output-dir'] ? resolve(values['output-dir']) :
  fileURLToPath(new URL('../output', import.meta.url)));
const root = fileURLToPath(new URL('./dist', import.meta.url));
if (!existsSync(join(root, 'index.html'))) throw new Error('Run npm run build before starting Motion Board.');
const host = '127.0.0.1';
const origin = `http://${host}:${localPort}`;
const media = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript', '.css': 'text/css' };
function permitted(req) {
  return req.headers.host === `${host}:${localPort}` &&
    (!req.headers.origin || req.headers.origin === origin);
}
const server = createServer((req, res) => {
  if (!permitted(req)) return res.writeHead(403).end();
  const path = new URL(req.url, origin).pathname;
  if (path === '/local-results' || path.startsWith('/local-results/')) {
    if (req.method !== 'GET') return res.writeHead(405, { Allow: 'GET' }).end();
    void results(path, res);
    return;
  }
  const asset = path === '/' ? '/index.html' : path;
  if (!/^\/(index\.html|assets\/[\w.-]+)$/.test(asset)) return res.writeHead(404).end();
  const file = join(root, asset);
  if (!existsSync(file) || !statSync(file).isFile()) return res.writeHead(404).end();
  res.writeHead(200, { 'Content-Type': media[extname(file)] || 'application/octet-stream' });
  createReadStream(file).pipe(res);
});
server.on('error', error => { console.error(error.message); process.exitCode = 1; });
server.listen(localPort, host, () => console.log(`Motion Board is ready at ${origin}`));
