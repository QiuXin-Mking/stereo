from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time


HTML_PAGE = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>RK3588 双目标定</title>
  <style>
    :root { color-scheme: dark; font-family: -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; }
    body { margin:0; background:#0b1020; color:#e5e7eb; }
    main { max-width:1500px; margin:auto; padding:18px; }
    h1 { margin:0 0 12px; font-size:24px; }
    .top,.metrics,.controls { display:flex; gap:10px; flex-wrap:wrap; margin:10px 0; }
    .card { background:#172033; border:1px solid #334155; border-radius:10px; padding:10px 14px; min-width:150px; }
    .card b { display:block; color:#60a5fa; font-size:18px; margin-top:3px; }
    #preview { width:100%; background:#111827; border:1px solid #334155; border-radius:12px; display:block; }
    #guidance { font-size:20px; text-align:center; background:#172033; border:2px solid #60a5fa; border-radius:10px; padding:12px; }
    progress { width:100%; height:16px; }
    button { border:0; border-radius:8px; padding:10px 17px; font-size:15px; color:white; background:#2563eb; cursor:pointer; }
    button.danger { background:#b91c1c; }
    #message { min-height:24px; color:#fbbf24; }
    .pass { color:#34d399 !important; } .error { color:#f87171 !important; }
  </style>
</head>
<body><main>
  <h1>RK3588 · SBS 双目棋盘格标定</h1>
  <div class="top">
    <div class="card">状态<b id="state">连接中</b></div>
    <div class="card">设备模式<b id="mode">-</b></div>
    <div class="card">单眼分辨率<b id="perEye">-</b></div>
    <div class="card">采集进度<b id="count">0 / 32</b></div>
  </div>
  <div id="guidance">等待状态...</div>
  <img id="preview" alt="双目实时预览">
  <div class="card"><span id="reason">等待棋盘</span><progress id="stable" max="1" value="0"></progress></div>
  <div class="metrics">
    <div class="card">左 RMS<b id="rmsL">-</b></div>
    <div class="card">右 RMS<b id="rmsR">-</b></div>
    <div class="card">极线 P95<b id="epi">-</b></div>
    <div class="card">结果目录<b id="result">-</b></div>
  </div>
  <div class="controls">
    <button onclick="act('pause')">暂停</button>
    <button onclick="act('resume')">继续</button>
    <button onclick="act('undo')">撤销上一对</button>
    <button onclick="act('solve')">开始求解</button>
    <button class="danger" onclick="act('stop')">停止服务</button>
  </div>
  <div id="message"></div>
</main>
<script>
const show = v => (v === null || v === undefined) ? '-' : v;
async function refresh() {
  try {
    const s = await (await fetch('/api/status', {cache:'no-store'})).json();
    state.textContent=s.state; state.className=(s.state==='pass'?'pass':(s.state==='error'||s.state==='retake'?'error':''));
    mode.textContent=s.mode; perEye.textContent=s.per_eye; count.textContent=`${s.accepted_pairs} / ${s.target_pairs}`;
    guidance.textContent=s.guidance; reason.textContent=s.error || s.reason; stable.value=s.stable_progress;
    rmsL.textContent=show(s.mono_rms_left); rmsR.textContent=show(s.mono_rms_right); epi.textContent=show(s.epipolar_p95); result.textContent=show(s.result_dir);
  } catch(e) { message.textContent='状态连接失败: '+e; }
}
async function act(action) {
  const response=await fetch('/api/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action})});
  const data=await response.json(); message.textContent=data.ok ? `操作成功: ${action}` : data.error; refresh();
}
window.addEventListener('load', () => {
  document.getElementById('preview').src='/stream.mjpg';
  refresh();
  setInterval(refresh,1000);
});
</script></body></html>"""


def create_server(engine, host: str, port: int) -> ThreadingHTTPServer:
    class CalibrationHandler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, _format, *_args):
            return

        def _send_bytes(self, status: int, content_type: str, body: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_json(self, status: int, payload) -> None:
            self._send_bytes(
                status,
                "application/json; charset=utf-8",
                json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            )

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/":
                self._send_bytes(HTTPStatus.OK, "text/html; charset=utf-8", HTML_PAGE.encode("utf-8"))
                return
            if path == "/api/status":
                self._send_json(HTTPStatus.OK, engine.status_snapshot())
                return
            if path == "/stream.mjpg":
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    while True:
                        frame = engine.latest_preview()
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                        if engine.status_snapshot().get("state") == "stopped":
                            return
                        time.sleep(0.10)
                except (BrokenPipeError, ConnectionResetError):
                    return
            self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "接口不存在"})

        def do_POST(self):
            if self.path.split("?", 1)[0] != "/api/action":
                self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "接口不存在"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > 4096:
                    raise ValueError("请求长度无效")
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
                action = payload.get("action")
                if not isinstance(action, str):
                    raise ValueError("缺少 action")
            except (ValueError, UnicodeDecodeError, json.JSONDecodeError) as error:
                self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(error)})
                return
            result = engine.action(action)
            status = HTTPStatus.OK if result.get("ok") else HTTPStatus.BAD_REQUEST
            self._send_json(status, result)
            if action == "stop" and result.get("ok"):
                threading.Thread(target=self.server.shutdown, daemon=True).start()

    server = ThreadingHTTPServer((host, port), CalibrationHandler)
    server.daemon_threads = True
    return server
