#!/usr/bin/env node
/**
 * 仿真平台本地服务器（零依赖，Node >= 18）
 *
 *   node server.mjs [端口]        默认 http://127.0.0.1:8000
 *
 * 功能：
 *   - 静态托管 web/、sim/、固件构建产物
 *   - GET  /api/params            读取 sim/core/cdpr_params.json
 *   - POST /api/params            写入参数（页面「参数编辑」保存用）
 *   - POST /api/build             编译 STM32 控制代码（或转为无板仿真），返回日志
 *   - GET  /api/health            健康检查
 *
 * 安全：只允许访问项目目录内的文件；写入仅限参数文件。
 */
import http from 'node:http';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { spawn } from 'node:child_process';
import crypto from 'node:crypto';

const ROOT = path.dirname(fileURLToPath(import.meta.url));
const PORT = Number(process.argv[2] || process.env.PORT || 8000);
const HOST = process.env.HOST || '0.0.0.0';

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.svg': 'image/svg+xml',
  '.ico': 'image/x-icon',
  '.map': 'application/json',
  '.md': 'text/markdown; charset=utf-8',
  '.bin': 'application/octet-stream',
};

// 页面以 /web/index.html 形式托管，其后相对路径引用以 /web/ 为根；
// 同时允许 /sim 与 /firmware 供 worker / 固件运行时导入。
const ALLOWED_ROOTS = ['web', 'sim', 'firmware', 'docs', 'build'];

function safePath(urlPath) {
  const clean = decodeURIComponent(urlPath.split('?')[0]).replace(/\\/g, '/');
  const rel = clean === '/' ? 'web/index.html' : clean.replace(/^\/+/, '');
  const abs = path.resolve(ROOT, rel);
  if (!abs.startsWith(ROOT + path.sep)) return null;
  const top = path.relative(ROOT, abs).split(path.sep)[0];
  if (clean !== '/' && !ALLOWED_ROOTS.includes(top)) return null;
  return abs;
}

function send(res, code, body, type = 'text/plain; charset=utf-8') {
  res.writeHead(code, {
    'Content-Type': type,
    'Cache-Control': 'no-store',
    'Access-Control-Allow-Origin': '*',
    'Cross-Origin-Opener-Policy': 'same-origin',
    'Cross-Origin-Embedder-Policy': 'require-corp',
  });
  res.end(body);
}

function readBody(req, limit = 4 * 1024 * 1024) {
  return new Promise((resolve, reject) => {
    let size = 0;
    const chunks = [];
    req.on('data', (c) => {
      size += c.length;
      if (size > limit) reject(new Error('请求体过大'));
      else chunks.push(c);
    });
    req.on('end', () => resolve(Buffer.concat(chunks).toString('utf8')));
    req.on('error', reject);
  });
}

function runBuild() {
  return new Promise((resolve) => {
    const isWin = process.platform === 'win32';
    const make = isWin ? 'mingw32-make' : 'make';
    const dir = path.join(ROOT, 'firmware');
    if (!fs.existsSync(dir)) {
      resolve({ ok: false, log: `未找到 firmware 目录：${dir}` });
      return;
    }
    const child = spawn(make, ['-j4'], { cwd: dir, shell: isWin });
    let out = '';
    const t = setTimeout(() => {
      child.kill();
      resolve({ ok: false, log: out + '\n[构建超时 120 s]' });
    }, 120000);
    child.stdout.on('data', (d) => { out += d.toString(); });
    child.stderr.on('data', (d) => { out += d.toString(); });
    child.on('error', (e) => {
      clearTimeout(t);
      resolve({
        ok: false,
        log: out + `\n无法启动编译器（${make}）：${e.message}\n` +
          '请安装 MinGW-w64 并加入 PATH（例如 C:\\mingw64\\bin），或在 MSYS2 中执行 pacman -S mingw-w64-x86_64-gcc。',
      });
    });
    child.on('close', (code) => {
      clearTimeout(t);
      resolve({ ok: code === 0, log: out, code });
    });
  });
}

// ---------------------------------------------------------------------------
// 固件无板仿真：启动 board_sim.exe，把 stdio JSON 广播给页面
// ---------------------------------------------------------------------------
const FW_EXE = path.join(ROOT, 'firmware', 'build', 'board_sim.exe');
let fwProc = null;
const wsClients = new Set();

function broadcast(obj) {
  const text = JSON.stringify(obj);
  for (const ws of wsClients) {
    if (ws.readyState === 1) ws.send(text);
  }
}

function startFirmware() {
  if (fwProc) return { ok: true, wsUrl: '', exe: FW_EXE, reused: true };
  if (!fs.existsSync(FW_EXE)) {
    return {
      ok: false,
      error: `未找到 ${FW_EXE}。请先运行 firmware\\build_sim.cmd（MSVC）或 firmware 目录下 make（MinGW）。`,
    };
  }
  fwProc = spawn(FW_EXE, [], { cwd: path.join(ROOT, 'firmware'), stdio: ['pipe', 'pipe', 'pipe'] });
  let buf = '';
  fwProc.stdout.on('data', (d) => {
    buf += d.toString('utf8');
    let idx;
    while ((idx = buf.indexOf('\n')) >= 0) {
      const line = buf.slice(0, idx).trim();
      buf = buf.slice(idx + 1);
      if (!line) continue;
      try {
        const msg = JSON.parse(line);
        if (msg.type === 'ready') broadcast({ type: 'fw-log', text: `固件就绪（${msg.rate} Hz，${msg.cables} 轴）` });
        else if (msg.type === 'cmd') broadcast({ ...msg, type: 'fw-cmd' });
        else if (msg.type === 'log') broadcast({ type: 'fw-log', text: msg.text });
      } catch {
        broadcast({ type: 'fw-log', text: line });
      }
    }
  });
  fwProc.stderr.on('data', (d) => broadcast({ type: 'fw-log', text: `[stderr] ${d.toString().trim()}` }));
  fwProc.on('exit', () => {
    fwProc = null;
    broadcast({ type: 'fw-exit' });
  });
  return { ok: true, exe: FW_EXE, wsUrl: '' };
}

function stopFirmware() {
  if (fwProc) {
    try { fwProc.stdin.write('{"cmd":"quit"}\n'); } catch { /* ignore */ }
    setTimeout(() => { try { fwProc && fwProc.kill(); } catch { /* ignore */ } }, 200);
    fwProc = null;
  }
  return { ok: true };
}

/** 极简 WebSocket 服务端（只用于把固件输出推给页面，不做通用实现） */
function attachWebSocket(httpServer) {
  httpServer.on('upgrade', (req, socket) => {
    if (!req.url.startsWith('/ws')) {
      socket.destroy();
      return;
    }
    const key = req.headers['sec-websocket-key'];
    const accept = crypto.createHash('sha1')
      .update(key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').digest('base64');
    socket.write(
      'HTTP/1.1 101 Switching Protocols\r\n'
      + 'Upgrade: websocket\r\n'
      + 'Connection: Upgrade\r\n'
      + `Sec-WebSocket-Accept: ${accept}\r\n\r\n`,
    );
    const client = { readyState: 1, send: (text) => writeFrame(socket, text) };
    wsClients.add(client);
    socket.on('data', (data) => {
      // 只解析文本帧（opcode 1）里的命令，转给固件进程
      const opcode = data[0] & 0x0f;
      if (opcode !== 1) return;
      let len = data[1] & 0x7f;
      let off = 2;
      if (len === 126) { len = data.readUInt16BE(2); off = 4; }
      const mask = data.slice(off, off + 4);
      off += 4;
      const payload = Buffer.alloc(len);
      for (let i = 0; i < len; i++) payload[i] = data[off + i] ^ mask[i % 4];
      try {
        const msg = JSON.parse(payload.toString('utf8'));
        if (fwProc && fwProc.stdin.writable) {
          const line = msg.type === 'goto' ? { cmd: 'goto', q: msg.q }
            : msg.type === 'circle' ? { cmd: 'circle' }
              : msg.type === 'pallet' ? { cmd: 'pallet' }
                : msg.type === 'hold' ? { cmd: 'hold' }
                  : { cmd: 'step', n: Math.min(200, Math.max(1, msg.n || 1)) };
          fwProc.stdin.write(JSON.stringify(line) + '\n');
        }
      } catch { /* ignore */ }
    });
    socket.on('close', () => wsClients.delete(client));
    socket.on('error', () => wsClients.delete(client));
  });
}

function writeFrame(socket, text) {
  const payload = Buffer.from(text, 'utf8');
  const len = payload.length;
  let header;
  if (len < 126) {
    header = Buffer.from([0x81, len]);
  } else if (len < 65536) {
    header = Buffer.alloc(4);
    header[0] = 0x81; header[1] = 126; header.writeUInt16BE(len, 2);
  } else {
    header = Buffer.alloc(10);
    header[0] = 0x81; header[1] = 127;
    header.writeUInt32BE(0, 2); header.writeUInt32BE(len, 6);
  }
  try { socket.write(Buffer.concat([header, payload])); } catch { /* ignore */ }
}

const server = http.createServer(async (req, res) => {
  const url = req.url || '/';
  try {
    // 页面内引用一律写成相对路径（为了能直接放到 GitHub Pages 这类纯静态托管上），
    // 所以根路径必须重定向到 /web/index.html，否则相对路径会解析到站点根目录。
    if (url === '/' || url === '/index.html') {
      res.writeHead(302, { Location: '/web/index.html' });
      res.end();
      return;
    }
    if (url.startsWith('/api/health')) {
      send(res, 200, JSON.stringify({ ok: true, pid: process.pid, node: process.version }), MIME['.json']);
      return;
    }
    if (url.startsWith('/api/fw')) {
      if (req.method === 'POST') {
        const r = startFirmware();
        send(res, r.ok ? 200 : 500, JSON.stringify({ ...r, wsUrl: `ws://127.0.0.1:${PORT}/ws` }), MIME['.json']);
        return;
      }
      if (req.method === 'DELETE') {
        send(res, 200, JSON.stringify(stopFirmware()), MIME['.json']);
        return;
      }
    }
    if (url.startsWith('/api/build') && req.method === 'POST') {
      const r = await runBuild();
      send(res, r.ok ? 200 : 500, JSON.stringify(r), MIME['.json']);
      return;
    }
    if (url.startsWith('/api/params')) {
      const file = path.join(ROOT, 'sim', 'core', 'cdpr_params.json');
      if (req.method === 'GET') {
        send(res, 200, fs.readFileSync(file, 'utf8'), MIME['.json']);
        return;
      }
      if (req.method === 'POST') {
        const body = await readBody(req);
        const json = JSON.parse(body);
        // 简单校验：必须是对象且含主要分节
        for (const k of ['frames', 'platform', 'winders', 'cables', 'actuators', 'controller']) {
          if (!json[k]) throw new Error(`缺少分节 ${k}`);
        }
        fs.writeFileSync(file, JSON.stringify(json, null, 2) + '\n', 'utf8');
        send(res, 200, JSON.stringify({ ok: true, bytes: body.length }), MIME['.json']);
        return;
      }
    }
    if (req.method !== 'GET' && req.method !== 'HEAD') {
      send(res, 405, 'method not allowed');
      return;
    }
    const abs = safePath(url);
    if (!abs) {
      send(res, 403, 'forbidden');
      return;
    }
    if (!fs.existsSync(abs) || fs.statSync(abs).isDirectory()) {
      const index = path.join(abs, 'index.html');
      if (fs.existsSync(index)) {
        send(res, 200, fs.readFileSync(index), MIME['.html']);
        return;
      }
      send(res, 404, `not found: ${url}`);
      return;
    }
    const ext = path.extname(abs).toLowerCase();
    send(res, 200, fs.readFileSync(abs), MIME[ext] || 'application/octet-stream');
  } catch (e) {
    send(res, 500, `server error: ${e.message}`);
  }
});

attachWebSocket(server);

function lanIP() {
  try {
    const ifaces = os.networkInterfaces();
    for (const name of Object.keys(ifaces)) {
      for (const ni of ifaces[name] || []) {
        if (ni.family === 'IPv4' && !ni.internal && !ni.address.startsWith('169.254')) {
          return ni.address;
        }
      }
    }
  } catch { /* ignore */ }
  return '127.0.0.1';
}

server.listen(PORT, HOST, () => {
  const url = `http://${HOST}:${PORT}/`;
  console.log(`绳驱并联机器人仿真平台已启动：${url}`);
  console.log(`  局域网访问：http://${lanIP()}:${PORT}/`);
  console.log('  页面：web/index.html（WebGL2 三维 + 实时曲线）');
  console.log('  接口：/api/params  /api/build  /api/health');
});
