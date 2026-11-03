/**
 * 「STM32 固件 · 无板仿真」数据源。
 *
 * 通过 /api/fw 启动并连接 firmware/build/board_sim.exe（同一份控制代码在 PC 上运行），
 * 由服务器把 stdio JSON 转成 WebSocket 广播给页面。
 *
 * 页面收到的消息：
 *   {type:'fw-log', text}                       固件日志
 *   {type:'fw-cmd', t, q, len, trq, feasible}   每批 step 后的控制输出
 */
export async function startFirmwareSim({ onLog, onCmd }) {
  const info = await fetch('/api/fw', { method: 'POST' }).then((r) => r.json()).catch(() => null);
  if (!info || !info.ok) {
    throw new Error(info && info.error ? info.error : '固件仿真进程启动失败');
  }
  const ws = new WebSocket(info.wsUrl);
  ws.onopen = () => onLog?.('info', `已连接固件仿真进程（${info.exe}）`);
  ws.onmessage = (ev) => {
    let msg;
    try {
      msg = JSON.parse(ev.data);
    } catch {
      return;
    }
    if (msg.type === 'fw-log') onLog?.('info', `[固件] ${msg.text}`);
    else if (msg.type === 'fw-cmd') onCmd?.(msg);
    else if (msg.type === 'fw-exit') onLog?.('warn', '固件仿真进程已退出');
  };
  ws.onerror = () => onLog?.('error', '固件仿真 WebSocket 错误');
  return {
    send: (cmd) => {
      if (ws.readyState === 1) ws.send(JSON.stringify(cmd));
    },
    stop: () => {
      fetch('/api/fw', { method: 'DELETE' }).catch(() => {});
      ws.close();
    },
  };
}
