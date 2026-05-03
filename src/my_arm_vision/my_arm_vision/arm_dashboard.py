#!/usr/bin/env python3
"""
arm_dashboard.py — 机械臂控制仪表盘

基于 Flask 的 Web 仪表盘，提供：
  · 实时摄像头检测画面（MJPEG 流）
  · 目标颜色选择（红 / 绿 / 蓝 / 黄 / 橙）
  · 抓取任务启动 / 停止
  · 当前状态实时显示

启动方式：
  ros2 run my_arm_vision arm_dashboard

然后在浏览器打开：
  http://<Raspberry-Pi-IP>:5000

依赖：
  pip install flask --break-system-packages
"""

import threading
from typing import Generator

import cv2
import rclpy
from cv_bridge import CvBridge
from flask import Flask, Response, jsonify, render_template_string, request
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

# ── HTML 模板 ─────────────────────────────────────────────────────────────────

HTML = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>MySelfArm Dashboard</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    background: #0f0f1a;
    color: #e0e0f0;
    font-family: 'Segoe UI', Arial, sans-serif;
    min-height: 100vh;
  }

  header {
    background: #1a1a30;
    padding: 14px 28px;
    display: flex;
    align-items: center;
    gap: 14px;
    border-bottom: 1px solid #2a2a50;
  }
  header h1 {
    font-size: 1.3rem;
    font-weight: 600;
    letter-spacing: 1px;
  }
  header .subtitle {
    font-size: 0.75rem;
    color: #8080b0;
  }

  .layout {
    display: flex;
    gap: 20px;
    padding: 20px;
    flex-wrap: wrap;
  }

  /* ── 视频区 ── */
  .video-panel {
    flex: 2;
    min-width: 320px;
  }
  .video-panel img {
    width: 100%;
    border-radius: 10px;
    border: 1px solid #2a2a50;
    display: block;
    background: #111;
  }
  .video-label {
    font-size: 0.72rem;
    color: #606090;
    margin-top: 6px;
    text-align: center;
  }

  /* ── 控制区 ── */
  .control-panel {
    flex: 1;
    min-width: 240px;
    display: flex;
    flex-direction: column;
    gap: 18px;
  }

  .card {
    background: #1a1a30;
    border: 1px solid #2a2a50;
    border-radius: 10px;
    padding: 16px;
  }
  .card h2 {
    font-size: 0.8rem;
    text-transform: uppercase;
    letter-spacing: 1.2px;
    color: #7070a0;
    margin-bottom: 12px;
  }

  /* 状态徽章 */
  .state-badge {
    display: inline-block;
    padding: 6px 14px;
    border-radius: 20px;
    font-size: 0.95rem;
    font-weight: 600;
    letter-spacing: 0.5px;
    transition: background 0.3s;
  }
  .state-IDLE            { background: #303050; color: #a0a0c0; }
  .state-SEARCHING       { background: #003050; color: #00cfff; }
  .state-APPROACHING     { background: #003020; color: #00ff80; }
  .state-DESCENDING      { background: #003020; color: #00ff80; }
  .state-GRASPING        { background: #004000; color: #80ff80; }
  .state-LIFTING         { background: #004000; color: #80ff80; }
  .state-MOVING_TO_PLACE { background: #403000; color: #ffd060; }
  .state-PLACING         { background: #403000; color: #ffd060; }
  .state-RELEASING       { background: #202060; color: #a0a0ff; }
  .state-HOMING          { background: #303030; color: #c0c0c0; }

  .target-info {
    margin-top: 8px;
    font-size: 0.8rem;
    color: #9090b0;
  }
  .target-info span {
    font-weight: 700;
    color: #e0e0ff;
  }

  /* 颜色选择按钮 */
  .color-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
  }
  .color-btn {
    padding: 10px 6px;
    border-radius: 8px;
    border: 2px solid transparent;
    cursor: pointer;
    font-size: 0.85rem;
    font-weight: 600;
    transition: all 0.15s;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 6px;
  }
  .color-btn:hover { opacity: 0.85; transform: scale(1.04); }
  .color-btn.active { border-color: #fff; box-shadow: 0 0 10px rgba(255,255,255,0.3); }

  .btn-red    { background: #7f1010; color: #ffb0b0; }
  .btn-green  { background: #0f6020; color: #a0ffb0; }
  .btn-blue   { background: #10207f; color: #a0c0ff; }
  .btn-yellow { background: #706010; color: #ffe080; }
  .btn-orange { background: #7f3010; color: #ffcf90; }

  /* 操作按钮 */
  .action-btns {
    display: flex;
    gap: 10px;
  }
  .btn-start, .btn-stop {
    flex: 1;
    padding: 12px;
    border-radius: 8px;
    border: none;
    font-size: 1rem;
    font-weight: 700;
    cursor: pointer;
    transition: opacity 0.15s, transform 0.1s;
    letter-spacing: 0.5px;
  }
  .btn-start:hover, .btn-stop:hover {
    opacity: 0.85;
    transform: translateY(-1px);
  }
  .btn-start { background: #1a7f30; color: #e0ffe0; }
  .btn-stop  { background: #7f1a1a; color: #ffe0e0; }

  /* 提示 */
  .tip {
    font-size: 0.73rem;
    color: #606080;
    line-height: 1.5;
  }
  .tip code {
    background: #0f0f20;
    padding: 1px 5px;
    border-radius: 4px;
    color: #a0c0ff;
  }

  /* 响应提示 */
  #feedback {
    height: 24px;
    font-size: 0.8rem;
    color: #60c060;
    text-align: center;
    transition: opacity 0.5s;
  }
</style>
</head>
<body>

<header>
  <svg width="28" height="28" viewBox="0 0 24 24" fill="none"
       stroke="#7080ff" stroke-width="2">
    <circle cx="12" cy="12" r="10"/>
    <path d="M12 8v4l3 3"/>
  </svg>
  <div>
    <h1>MySelfArm Dashboard</h1>
    <div class="subtitle">树莓派 × ESP32 × ROS2 视觉抓取控制台</div>
  </div>
</header>

<div class="layout">

  <!-- 视频流 -->
  <div class="video-panel">
    <img id="stream" src="/video_feed" alt="Camera stream" />
    <div class="video-label">实时检测画面 · /arm_debug_image</div>
  </div>

  <!-- 控制面板 -->
  <div class="control-panel">

    <!-- 状态卡 -->
    <div class="card">
      <h2>当前状态</h2>
      <span id="state-badge" class="state-badge state-IDLE">IDLE</span>
      <div class="target-info">
        目标颜色：<span id="cur-color">—</span>
      </div>
    </div>

    <!-- 颜色选择卡 -->
    <div class="card">
      <h2>选择目标颜色</h2>
      <div class="color-grid">
        <button class="color-btn btn-red    active" onclick="setColor('red')">
          🔴 红色
        </button>
        <button class="color-btn btn-green"  onclick="setColor('green')">
          🟢 绿色
        </button>
        <button class="color-btn btn-blue"   onclick="setColor('blue')">
          🔵 蓝色
        </button>
        <button class="color-btn btn-yellow" onclick="setColor('yellow')">
          🟡 黄色
        </button>
        <button class="color-btn btn-orange" onclick="setColor('orange')">
          🟠 橙色
        </button>
      </div>
    </div>

    <!-- 操作卡 -->
    <div class="card">
      <h2>抓取控制</h2>
      <div class="action-btns">
        <button class="btn-start" onclick="doAction('/api/start')">▶ 开始抓取</button>
        <button class="btn-stop"  onclick="doAction('/api/stop')"> ■ 停止</button>
      </div>
      <div id="feedback" style="margin-top:10px;"></div>
    </div>

    <!-- 提示卡 -->
    <div class="card">
      <h2>命令行控制</h2>
      <div class="tip">
        也可通过终端控制：<br>
        <code>ros2 topic pub /start_grasp std_msgs/msg/Bool "data: true" --once</code><br><br>
        切换颜色：<br>
        <code>ros2 topic pub /set_target_color std_msgs/msg/String "data: 'green'" --once</code>
      </div>
    </div>

  </div>
</div>

<script>
let currentColor = 'red';

function setColor(color) {
  fetch('/api/color/' + color, { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      showFeedback('颜色已切换为: ' + color);
      document.querySelectorAll('.color-btn').forEach(b => b.classList.remove('active'));
      document.querySelector('.btn-' + color)?.classList.add('active');
      currentColor = color;
    });
}

function doAction(url) {
  fetch(url, { method: 'POST' })
    .then(r => r.json())
    .then(d => showFeedback(d.message || 'OK'));
}

function showFeedback(msg) {
  const el = document.getElementById('feedback');
  el.style.opacity = 1;
  el.textContent = msg;
  setTimeout(() => { el.style.opacity = 0; }, 2500);
}

function refreshState() {
  fetch('/api/status')
    .then(r => r.json())
    .then(d => {
      const badge = document.getElementById('state-badge');
      badge.className = 'state-badge state-' + d.state;
      badge.textContent = d.state;
      document.getElementById('cur-color').textContent = d.color || '—';
    })
    .catch(() => {});
}

setInterval(refreshState, 500);
refreshState();
</script>
</body>
</html>
"""


# ── ROS2 + Flask 节点 ──────────────────────────────────────────────────────────

class ArmDashboardNode(Node):

    def __init__(self):
        super().__init__('arm_dashboard')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 5000)

        self._host  = self.get_parameter('host').value
        self._port  = self.get_parameter('port').value
        self._bridge = CvBridge()

        # 最新帧（JPEG 字节）
        self._frame_lock   = threading.Lock()
        self._latest_frame: bytes = b''

        # 最新状态字符串 "STATE|color"
        self._state_lock = threading.Lock()
        self._state_str  = 'IDLE|red'

        # ── ROS 订阅 / 发布 ──────────────────────────────────────
        self.create_subscription(Image,  '/arm_debug_image', self._img_cb,   10)
        self.create_subscription(String, '/arm_state',       self._state_cb, 10)

        self._start_pub = self.create_publisher(Bool,   '/start_grasp',      10)
        self._stop_pub  = self.create_publisher(Bool,   '/stop_grasp',       10)
        self._color_pub = self.create_publisher(String, '/set_target_color', 10)

        # ── 启动 Flask ────────────────────────────────────────────
        self._app = self._build_app()
        flask_thread = threading.Thread(
            target=lambda: self._app.run(
                host=self._host, port=self._port,
                debug=False, use_reloader=False),
            daemon=True)
        flask_thread.start()

        self.get_logger().info('━' * 48)
        self.get_logger().info('  arm_dashboard 已启动')
        self.get_logger().info(f'  浏览器打开: http://<IP>:{self._port}')
        self.get_logger().info('━' * 48)

    # ── ROS 回调 ──────────────────────────────────────────────────────────────

    def _img_cb(self, msg: Image):
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            _, buf = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with self._frame_lock:
                self._latest_frame = buf.tobytes()
        except Exception:
            pass

    def _state_cb(self, msg: String):
        with self._state_lock:
            self._state_str = msg.data

    # ── Flask ─────────────────────────────────────────────────────────────────

    def _build_app(self) -> Flask:
        app = Flask(__name__)
        node = self  # closure

        @app.route('/')
        def index():
            return render_template_string(HTML)

        @app.route('/video_feed')
        def video_feed():
            return Response(
                node._mjpeg_stream(),
                mimetype='multipart/x-mixed-replace; boundary=frame')

        @app.route('/api/status')
        def status():
            with node._state_lock:
                raw = node._state_str
            parts = raw.split('|')
            return jsonify({'state': parts[0], 'color': parts[1] if len(parts) > 1 else ''})

        @app.route('/api/start', methods=['POST'])
        def start():
            msg = Bool(); msg.data = True
            node._start_pub.publish(msg)
            return jsonify({'message': '已发送开始指令'})

        @app.route('/api/stop', methods=['POST'])
        def stop():
            msg = Bool(); msg.data = True
            node._stop_pub.publish(msg)
            return jsonify({'message': '已发送停止指令'})

        @app.route('/api/color/<color>', methods=['POST'])
        def set_color(color):
            valid = ['red', 'green', 'blue', 'yellow', 'orange']
            if color not in valid:
                return jsonify({'error': f'无效颜色，可选: {valid}'}), 400
            msg = String(); msg.data = color
            node._color_pub.publish(msg)
            return jsonify({'message': f'颜色已设为 {color}'})

        return app

    def _mjpeg_stream(self) -> Generator[bytes, None, None]:
        """生成 MJPEG 流"""
        placeholder = self._make_placeholder()
        while True:
            with self._frame_lock:
                frame = self._latest_frame if self._latest_frame else placeholder
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    @staticmethod
    def _make_placeholder() -> bytes:
        """无图像时的占位帧"""
        img = 10 * np.ones((240, 320, 3), dtype=np.uint8)
        cv2.putText(img, 'Waiting for camera...', (30, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 80, 120), 1)
        _, buf = cv2.imencode('.jpg', img)
        return buf.tobytes()


# ── 入口 ──────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ArmDashboardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
