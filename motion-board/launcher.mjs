import { spawn } from 'node:child_process';
import { createReadStream, existsSync, statSync } from 'node:fs';
import { createServer, request } from 'node:http';
import { join, extname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { parseArgs } from 'node:util';

const { values } = parseArgs({ options: {
  ssh: { type: 'string' }, 'api-port': { type: 'string', default: '5052' },
  port: { type: 'string', default: '8082' },
  'forward-port': { type: 'string', default: '15052' },
}});
function port(value) {
  const number = Number(value);
  if (!Number.isInteger(number) || number < 1 || number > 65535) throw new Error('Invalid port');
  return number;
}
const localPort = port(values.port);
const apiPort = port(values['api-port']);
const upstreamPort = values.ssh ? port(values['forward-port']) : apiPort;
if (upstreamPort === localPort) throw new Error('The console and upstream ports must be different.');
const root = fileURLToPath(new URL('./dist', import.meta.url));
if (!existsSync(join(root, 'index.html'))) throw new Error('Run npm run build before starting Motion Board.');
const host = '127.0.0.1';
const origin = `http://${host}:${localPort}`;
const tunnel = values.ssh && spawn('ssh', [
  '-NT', '-o', 'ExitOnForwardFailure=yes', '-o', 'BatchMode=yes',
  '-o', 'ServerAliveInterval=15', '-L', `${host}:${upstreamPort}:${host}:${apiPort}`, '--', values.ssh,
], { stdio: 'inherit' });

const media = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript', '.css': 'text/css' };
function permitted(req) {
  return req.headers.host === `${host}:${localPort}` &&
    (!req.headers.origin || req.headers.origin === origin);
}
function upstream(req) {
  const { origin: _origin, host: _host, ...headers } = req.headers;
  return request({ host, port: upstreamPort, path: req.url.slice(4), method: req.method, headers });
}
const server = createServer((req, res) => {
  if (!permitted(req)) return res.writeHead(403).end();
  if (req.url.startsWith('/api/')) {
    const proxy = upstream(req);
    proxy.on('response', response => {
      res.writeHead(response.statusCode, response.headers);
      response.pipe(res);
    });
    proxy.on('error', () => {
      if (!res.headersSent) res.writeHead(502, { 'Content-Type': 'application/json' }).end(
        '{"detail":"Experiment API is unreachable. Check the Pi service and SSH tunnel."}');
      else res.destroy();
    });
    res.on('close', () => { if (!res.writableFinished) proxy.destroy(); });
    req.pipe(proxy);
    return;
  }
  const path = new URL(req.url, origin).pathname;
  const asset = path === '/' ? '/index.html' : path;
  if (!/^\/(index\.html|assets\/[\w.-]+)$/.test(asset)) return res.writeHead(404).end();
  const file = join(root, asset);
  if (!existsSync(file) || !statSync(file).isFile()) return res.writeHead(404).end();
  res.writeHead(200, { 'Content-Type': media[extname(file)] || 'application/octet-stream' });
  createReadStream(file).pipe(res);
});
server.on('upgrade', (req, socket, head) => {
  if (!permitted(req) || req.url !== '/api/events') return socket.destroy();
  const proxy = upstream(req);
  socket.on('error', () => proxy.destroy());
  socket.on('close', () => proxy.destroy());
  proxy.on('error', () => socket.destroy());
  proxy.on('response', () => socket.destroy());
  proxy.on('upgrade', (response, remote, rest) => {
    socket.write(`HTTP/1.1 101 Switching Protocols\r\n${Object.entries(response.headers)
      .map(([key, value]) => `${key}: ${value}`).join('\r\n')}\r\n\r\n`);
    if (rest.length) socket.write(rest);
    if (head.length) remote.write(head);
    remote.on('error', () => socket.destroy());
    remote.on('close', () => socket.destroy());
    socket.on('close', () => remote.destroy());
    remote.pipe(socket).pipe(remote);
  });
  proxy.end();
});
tunnel?.on('error', error => console.error(error.message));
tunnel?.on('exit', code => { console.error(`SSH tunnel closed (${code}).`); process.exit(1); });
server.on('error', error => { console.error(error.message); tunnel?.kill(); process.exitCode = 1; });
server.listen(localPort, host, () => console.log(`Motion Board is ready at ${origin}`));
for (const signal of ['SIGINT', 'SIGTERM']) process.on(signal, () => { tunnel?.kill(); process.exit(0); });
