#!/usr/bin/env python3
"""
PID Dashboard — Web-based real-time PID visualization.

Run on the RPi alongside the straight_line_assistant node.
Open http://<rpi-ip>:8080 from any browser on your network.

Usage:
    ros2 run straight_line_assistant pid_dashboard
"""

import json
import threading
import time
from collections import deque
from http.server import HTTPServer, BaseHTTPRequestHandler

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


# ── Shared state between ROS node and HTTP server ────────────────────
_lock = threading.Lock()
_data_buffer = deque(maxlen=2000)  # ~100 seconds at 20Hz
_point_id = 0
_start_time = None


# ── Dashboard HTML (served as a single page) ─────────────────────────
DASHBOARD_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>PID Dashboard — Straight Line Assistant</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4"></script>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap');

  * { margin: 0; padding: 0; box-sizing: border-box; }

  body {
    font-family: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif;
    background: #0a0b10;
    color: #e0e0e0;
    min-height: 100vh;
  }

  .dashboard {
    max-width: 1200px;
    margin: 0 auto;
    padding: 24px;
  }

  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: 24px;
    padding-bottom: 16px;
    border-bottom: 1px solid #1e2030;
  }

  header h1 {
    font-size: 22px;
    font-weight: 700;
    background: linear-gradient(135deg, #667eea, #764ba2);
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
    background-clip: text;
  }

  .status {
    display: flex;
    align-items: center;
    gap: 8px;
    font-size: 13px;
    color: #888;
  }

  .status-dot {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: #444;
    transition: background 0.3s;
  }

  .status-dot.active {
    background: #4ade80;
    box-shadow: 0 0 8px #4ade8066;
  }

  .stats-row {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
    gap: 12px;
    margin-bottom: 24px;
  }

  .stat-card {
    background: #12141e;
    border: 1px solid #1e2030;
    border-radius: 12px;
    padding: 14px 16px;
    transition: border-color 0.3s;
  }

  .stat-card:hover { border-color: #2a2e45; }

  .stat-label {
    font-size: 11px;
    font-weight: 600;
    text-transform: uppercase;
    letter-spacing: 0.5px;
    color: #556;
    margin-bottom: 6px;
  }

  .stat-value {
    font-size: 20px;
    font-weight: 700;
    font-variant-numeric: tabular-nums;
  }

  .chart-container {
    background: #12141e;
    border: 1px solid #1e2030;
    border-radius: 12px;
    padding: 20px;
    margin-bottom: 16px;
  }

  .chart-title {
    font-size: 14px;
    font-weight: 600;
    color: #99a;
    margin-bottom: 12px;
  }

  .chart-wrap {
    position: relative;
    height: 240px;
  }

  .heading-hold {
    display: inline-block;
    padding: 2px 10px;
    border-radius: 6px;
    font-size: 13px;
    font-weight: 600;
  }
  .heading-hold.on  { background: #4ade8018; color: #4ade80; }
  .heading-hold.off { background: #f8717118; color: #f87171; }

  .footer-hint {
    text-align: center;
    color: #445;
    font-size: 12px;
    margin-top: 16px;
  }
</style>
</head>
<body>
<div class="dashboard">
  <header>
    <h1>&#9881; PID Dashboard</h1>
    <div class="status">
      <span class="status-dot" id="statusDot"></span>
      <span id="statusText">Connecting...</span>
    </div>
  </header>

  <div class="stats-row">
    <div class="stat-card">
      <div class="stat-label">Yaw Error</div>
      <div class="stat-value" style="color:#ff6b6b" id="valError">&mdash;</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Output &omega;</div>
      <div class="stat-value" style="color:#4ecdc4" id="valOutput">&mdash;</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Target Yaw</div>
      <div class="stat-value" style="color:#55efc4" id="valTarget">&mdash;</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Current Yaw</div>
      <div class="stat-value" style="color:#fd79a8" id="valCurrent">&mdash;</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">P term</div>
      <div class="stat-value" style="color:#74b9ff" id="valP">&mdash;</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">I term</div>
      <div class="stat-value" style="color:#a29bfe" id="valI">&mdash;</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">D term</div>
      <div class="stat-value" style="color:#fdcb6e" id="valD">&mdash;</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Heading Hold</div>
      <div class="stat-value" id="valHold">&mdash;</div>
    </div>
  </div>

  <div class="chart-container">
    <div class="chart-title">Yaw Error &amp; Angular Output</div>
    <div class="chart-wrap"><canvas id="chart1"></canvas></div>
  </div>

  <div class="chart-container">
    <div class="chart-title">PID Components</div>
    <div class="chart-wrap"><canvas id="chart2"></canvas></div>
  </div>

  <div class="footer-hint">
    Tune live: <code>ros2 param set /straight_line_assistant kp 1.0</code>
  </div>
</div>

<script>
const MAX_POINTS = 600;
let lastId = -1;
let lastDataTime = 0;

const chartOpts = (yLabel) => ({
  responsive: true,
  maintainAspectRatio: false,
  animation: { duration: 0 },
  interaction: { intersect: false, mode: 'index' },
  scales: {
    x: {
      display: true,
      title: { display: true, text: 'Time (s)', color: '#556' },
      ticks: { color: '#334', maxTicksLimit: 10, font: { size: 10 } },
      grid: { color: '#181a28' },
    },
    y: {
      display: true,
      title: { display: true, text: yLabel, color: '#556' },
      ticks: { color: '#334', font: { size: 10 } },
      grid: { color: '#181a28' },
    },
  },
  plugins: {
    legend: {
      labels: { color: '#99a', usePointStyle: true, pointStyleWidth: 12, font: { size: 11 } }
    },
  },
});

const ds = (label, color, fill) => ({
  label: label,
  data: [],
  borderColor: color,
  backgroundColor: fill ? color + '22' : 'transparent',
  borderWidth: 2,
  pointRadius: 0,
  fill: !!fill,
  tension: 0.3,
});

const chart1 = new Chart(document.getElementById('chart1'), {
  type: 'line',
  data: { labels: [], datasets: [
    ds('Yaw Error (rad)', '#ff6b6b', true),
    ds('Output ω (rad/s)', '#4ecdc4', true),
  ]},
  options: chartOpts('rad / rad·s⁻¹'),
});

const chart2 = new Chart(document.getElementById('chart2'), {
  type: 'line',
  data: { labels: [], datasets: [
    ds('P', '#74b9ff', false),
    ds('I', '#a29bfe', false),
    ds('D', '#fdcb6e', false),
  ]},
  options: chartOpts('Contribution'),
});

function trimChart(chart) {
  while (chart.data.labels.length > MAX_POINTS) {
    chart.data.labels.shift();
    chart.data.datasets.forEach(d => d.data.shift());
  }
}

async function poll() {
  try {
    const resp = await fetch('/api/data?since=' + lastId);
    const points = await resp.json();

    if (points.length > 0) {
      lastDataTime = Date.now();
      document.getElementById('statusDot').classList.add('active');
      document.getElementById('statusText').textContent =
        'Streaming — ' + points.length + ' pts';

      for (const p of points) {
        lastId = p.id;

        const tLabel = p.t.toFixed(1);
        chart1.data.labels.push(tLabel);
        chart1.data.datasets[0].data.push(p.error);
        chart1.data.datasets[1].data.push(p.output);

        chart2.data.labels.push(tLabel);
        chart2.data.datasets[0].data.push(p.p);
        chart2.data.datasets[1].data.push(p.i);
        chart2.data.datasets[2].data.push(p.d);
      }

      trimChart(chart1);
      trimChart(chart2);
      chart1.update();
      chart2.update();

      const lp = points[points.length - 1];
      document.getElementById('valError').textContent   = lp.error.toFixed(4);
      document.getElementById('valOutput').textContent  = lp.output.toFixed(4);
      document.getElementById('valTarget').textContent  = lp.target_yaw.toFixed(3);
      document.getElementById('valCurrent').textContent = lp.current_yaw.toFixed(3);
      document.getElementById('valP').textContent       = lp.p.toFixed(4);
      document.getElementById('valI').textContent       = lp.i.toFixed(4);
      document.getElementById('valD').textContent       = lp.d.toFixed(4);
      document.getElementById('valHold').innerHTML =
        '<span class="heading-hold on">ON</span>';
    } else {
      if (Date.now() - lastDataTime > 2000 && lastDataTime > 0) {
        document.getElementById('valHold').innerHTML =
          '<span class="heading-hold off">OFF</span>';
        document.getElementById('statusText').textContent =
          'Heading-hold inactive — drive straight to engage';
      }
    }
  } catch (e) {
    document.getElementById('statusDot').classList.remove('active');
    document.getElementById('statusText').textContent = 'Connection lost — retrying...';
  }
}

setInterval(poll, 100);
</script>
</body>
</html>"""


class PIDDashboardNode(Node):
    """ROS 2 node that subscribes to /pid_debug and serves a web dashboard."""

    def __init__(self):
        super().__init__('pid_dashboard')
        global _start_time
        _start_time = time.time()

        self.declare_parameter('port', 8080)
        self.port = self.get_parameter('port').value

        self.sub = self.create_subscription(
            Float32MultiArray, '/pid_debug', self.pid_cb, 10)

        self.get_logger().info(
            f'PID Dashboard starting on http://0.0.0.0:{self.port}')

    def pid_cb(self, msg):
        global _point_id
        if len(msg.data) < 7:
            return
        t = time.time() - _start_time
        point = {
            'id': _point_id,
            't': round(t, 3),
            'error': round(msg.data[0], 6),
            'p': round(msg.data[1], 6),
            'i': round(msg.data[2], 6),
            'd': round(msg.data[3], 6),
            'output': round(msg.data[4], 6),
            'target_yaw': round(msg.data[5], 4),
            'current_yaw': round(msg.data[6], 4),
        }
        with _lock:
            _data_buffer.append(point)
            _point_id += 1


class DashboardHandler(BaseHTTPRequestHandler):
    """Simple HTTP handler — serves the dashboard and a JSON data API."""

    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            self._serve_html()
        elif self.path.startswith('/api/data'):
            self._serve_data()
        else:
            self.send_error(404)

    def _serve_html(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.end_headers()
        self.wfile.write(DASHBOARD_HTML.encode('utf-8'))

    def _serve_data(self):
        # Parse ?since=<id>
        since_id = -1
        if '?' in self.path:
            qs = self.path.split('?', 1)[1]
            for part in qs.split('&'):
                if part.startswith('since='):
                    try:
                        since_id = int(part.split('=', 1)[1])
                    except ValueError:
                        pass

        with _lock:
            points = [p for p in _data_buffer if p['id'] > since_id]

        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(json.dumps(points).encode('utf-8'))

    def log_message(self, format, *args):
        pass  # Suppress HTTP request logs


def main(args=None):
    rclpy.init(args=args)
    node = PIDDashboardNode()

    # Start HTTP server in a daemon thread
    server = HTTPServer(('0.0.0.0', node.port), DashboardHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    node.get_logger().info(
        f'Dashboard ready — open http://<rpi-ip>:{node.port} in your browser')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
