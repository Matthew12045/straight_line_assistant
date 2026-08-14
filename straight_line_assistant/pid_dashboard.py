#!/usr/bin/env python3
"""
PID & EKF Web Dashboard — Real-time visualization, tuning control & CSV data export.

Features:
1. PID Dashboard with Rolling Mean Absolute Error (MAE) and parameter save capability.
2. EKF Comparison Dashboard (Raw Odom vs Raw IMU vs EKF Filtered).
3. Web interface with tab navigation and "Save Values" button to persist PID params.
4. CSV Data Export for PID & EKF telemetry logs.

Usage:
    ros2 run straight_line_assistant pid_dashboard
"""

import json
import math
import os
import threading
import time
from collections import deque
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import parse_qs, urlparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


# ── Shared State ─────────────────────────────────────────────────────
_lock = threading.Lock()
_start_time = None

_pid_data_buffer = deque(maxlen=5000)  # ~250 seconds of telemetry
_pid_point_id = 0
_recent_errors = deque(maxlen=100)

_ekf_data_buffer = deque(maxlen=5000)
_ekf_point_id = 0

_latest_odom_yaw = 0.0
_latest_odom_v = 0.0
_latest_odom_w = 0.0

_latest_imu_yaw = 0.0
_latest_imu_w = 0.0

_node_reference = None


def euler_from_quaternion(q):
    """Extract yaw angle from geometry_msgs/Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


# ── Web Interface HTML/JS (Single-page app with 2 tabs & CSV Export) ──
DASHBOARD_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Robot Control & Navigation Dashboard</title>
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
    max-width: 1280px;
    margin: 0 auto;
    padding: 24px;
  }

  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: 20px;
    padding-bottom: 16px;
    border-bottom: 1px solid #1e2030;
  }

  .header-title-group {
    display: flex;
    align-items: center;
    gap: 20px;
  }

  header h1 {
    font-size: 20px;
    font-weight: 700;
    background: linear-gradient(135deg, #667eea, #764ba2);
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
    background-clip: text;
  }

  .nav-tabs {
    display: flex;
    gap: 8px;
    background: #12141e;
    padding: 4px;
    border-radius: 8px;
    border: 1px solid #1e2030;
  }

  .tab-btn {
    background: transparent;
    border: none;
    color: #889;
    padding: 8px 16px;
    font-size: 13px;
    font-weight: 600;
    border-radius: 6px;
    cursor: pointer;
    transition: all 0.2s;
  }

  .tab-btn.active {
    background: #252a40;
    color: #fff;
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

  .tab-content {
    display: none;
  }

  .tab-content.active {
    display: block;
  }

  .stats-row {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(130px, 1fr));
    gap: 12px;
    margin-bottom: 20px;
  }

  .stat-card {
    background: #12141e;
    border: 1px solid #1e2030;
    border-radius: 12px;
    padding: 14px 16px;
  }

  .stat-card.highlight {
    border-color: #667eea;
    background: #151828;
  }

  .stat-label {
    font-size: 11px;
    font-weight: 600;
    text-transform: uppercase;
    letter-spacing: 0.5px;
    color: #556;
    margin-bottom: 6px;
  }

  .stat-value {
    font-size: 18px;
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
    display: flex;
    justify-content: space-between;
    align-items: center;
  }

  .chart-wrap {
    position: relative;
    height: 220px;
  }

  .action-bar {
    display: flex;
    align-items: center;
    justify-content: space-between;
    background: #12141e;
    border: 1px solid #1e2030;
    border-radius: 12px;
    padding: 16px 20px;
    margin-top: 16px;
  }

  .btn-group {
    display: flex;
    gap: 10px;
  }

  .btn {
    background: linear-gradient(135deg, #667eea, #764ba2);
    color: white;
    border: none;
    padding: 10px 18px;
    font-size: 13px;
    font-weight: 600;
    border-radius: 8px;
    cursor: pointer;
    transition: opacity 0.2s;

  }

  .btn-secondary {
    background: #252a40;
    color: #4ade80;
    border: 1px solid #4ade8044;
  }

  .btn:hover { opacity: 0.9; }

  .heading-hold {
    display: inline-block;
    padding: 2px 10px;
    border-radius: 6px;
    font-size: 12px;
    font-weight: 600;
  }
  .heading-hold.on  { background: #4ade8018; color: #4ade80; }
  .heading-hold.off { background: #f8717118; color: #f87171; }
</style>
</head>
<body>
<div class="dashboard">
  <header>
    <div class="header-title-group">
      <h1>&#9881; Robot Control Dashboard</h1>
      <div class="nav-tabs">
        <button class="tab-btn active" onclick="switchTab('pidTab')">PID Controller</button>
        <button class="tab-btn" onclick="switchTab('ekfTab')">EKF Sensor Fusion</button>
      </div>
    </div>
    <div class="status">
      <span class="status-dot" id="statusDot"></span>
      <span id="statusText">Connecting...</span>
    </div>
  </header>

  <!-- ── TAB 1: PID TUNING ────────────────────────────────────────── -->
  <div id="pidTab" class="tab-content active">
    <div class="stats-row">
      <div class="stat-card highlight">
        <div class="stat-label">Moving Abs Error</div>
        <div class="stat-value" style="color:#a855f7" id="valMAE">&mdash;</div>
      </div>
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
        <div class="stat-label">P Term</div>
        <div class="stat-value" style="color:#74b9ff" id="valP">&mdash;</div>
      </div>
      <div class="stat-card">
        <div class="stat-label">I Term</div>
        <div class="stat-value" style="color:#a29bfe" id="valI">&mdash;</div>
      </div>
      <div class="stat-card">
        <div class="stat-label">D Term</div>
        <div class="stat-value" style="color:#fdcb6e" id="valD">&mdash;</div>
      </div>
      <div class="stat-card">
        <div class="stat-label">Heading Hold</div>
        <div class="stat-value" id="valHold">&mdash;</div>
      </div>
    </div>

    <div class="chart-container">
      <div class="chart-title">Yaw Error &amp; Angular Output</div>
      <div class="chart-wrap"><canvas id="chartPid1"></canvas></div>
    </div>

    <div class="chart-container">
      <div class="chart-title">PID Components (P, I, D)</div>
      <div class="chart-wrap"><canvas id="chartPid2"></canvas></div>
    </div>

    <div class="action-bar">
      <div>
        <strong>PID Data Controls:</strong>
        <span style="color:#889; font-size:12px; margin-left:8px;">Save values to YAML or download telemetry log as CSV.</span>
      </div>
      <div class="btn-group">
        <button class="btn btn-secondary" onclick="window.location.href='/api/export_pid.csv'">📥 Export PID CSV</button>
        <button class="btn" onclick="savePIDValues()">💾 Save Values to YAML</button>
      </div>
    </div>
  </div>

  <!-- ── TAB 2: EKF SENSOR FUSION ─────────────────────────────────── -->
  <div id="ekfTab" class="tab-content">
    <div class="stats-row">
      <div class="stat-card">
        <div class="stat-label">Odom Yaw</div>
        <div class="stat-value" style="color:#38bdf8" id="valOdomYaw">&mdash;</div>
      </div>
      <div class="stat-card">
        <div class="stat-label">IMU Yaw</div>
        <div class="stat-value" style="color:#fbbf24" id="valImuYaw">&mdash;</div>
      </div>
      <div class="stat-card highlight">
        <div class="stat-label">EKF Filtered Yaw</div>
        <div class="stat-value" style="color:#4ade80" id="valEkfYaw">&mdash;</div>
      </div>
      <div class="stat-card">
        <div class="stat-label">IMU &omega; (rad/s)</div>
        <div class="stat-value" style="color:#fbbf24" id="valImuW">&mdash;</div>
      </div>
      <div class="stat-card">
        <div class="stat-label">EKF &omega; (rad/s)</div>
        <div class="stat-value" style="color:#4ade80" id="valEkfW">&mdash;</div>
      </div>
    </div>

    <div class="chart-container">
      <div class="chart-title">Yaw Angle Comparison (Odom vs IMU vs Filtered EKF)</div>
      <div class="chart-wrap"><canvas id="chartEkfYaw"></canvas></div>
    </div>

    <div class="chart-container">
      <div class="chart-title">Angular Velocity Comparison &omega; (Odom vs IMU vs Filtered EKF)</div>
      <div class="chart-wrap"><canvas id="chartEkfW"></canvas></div>
    </div>

    <div class="chart-container">
      <div class="chart-title">Linear Velocity Comparison v (Odom vs Filtered EKF)</div>
      <div class="chart-wrap"><canvas id="chartEkfV"></canvas></div>
    </div>

    <div class="action-bar">
      <div>
        <strong>EKF Sensor Telemetry Export:</strong>
        <span style="color:#889; font-size:12px; margin-left:8px;">Download raw vs filtered EKF sensor data as CSV.</span>
      </div>
      <button class="btn btn-secondary" onclick="window.location.href='/api/export_ekf.csv'">📥 Export EKF CSV</button>
    </div>
  </div>

</div>

<script>
const MAX_POINTS = 500;
let lastPidId = -1;
let lastEkfId = -1;

function switchTab(tabId) {
  document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
  document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));

  event.target.classList.add('active');
  document.getElementById(tabId).classList.add('active');
}

const chartOpts = (yLabel) => ({
  responsive: true,
  maintainAspectRatio: false,
  animation: { duration: 0 },
  interaction: { intersect: false, mode: 'index' },
  scales: {
    x: {
      display: true,
      ticks: { color: '#445', maxTicksLimit: 8, font: { size: 10 } },
      grid: { color: '#181a28' },
    },
    y: {
      display: true,
      title: { display: true, text: yLabel, color: '#556' },
      ticks: { color: '#445', font: { size: 10 } },
      grid: { color: '#181a28' },
    },
  },
  plugins: {
    legend: { labels: { color: '#99a', usePointStyle: true, font: { size: 11 } } }
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
  tension: 0.2,
});

// PID Charts
const chartPid1 = new Chart(document.getElementById('chartPid1'), {
  type: 'line',
  data: { labels: [], datasets: [ds('Yaw Error (rad)', '#ff6b6b', true), ds('Output ω (rad/s)', '#4ecdc4', true)] },
  options: chartOpts('rad / rad·s⁻¹'),
});

const chartPid2 = new Chart(document.getElementById('chartPid2'), {
  type: 'line',
  data: { labels: [], datasets: [ds('P', '#74b9ff'), ds('I', '#a29bfe'), ds('D', '#fdcb6e')] },
  options: chartOpts('Contribution'),
});

// EKF Charts
const chartEkfYaw = new Chart(document.getElementById('chartEkfYaw'), {
  type: 'line',
  data: { labels: [], datasets: [ds('Raw Odom Yaw', '#38bdf8'), ds('Raw IMU Yaw', '#fbbf24'), ds('EKF Filtered Yaw', '#4ade80')] },
  options: chartOpts('radians'),
});

const chartEkfW = new Chart(document.getElementById('chartEkfW'), {
  type: 'line',
  data: { labels: [], datasets: [ds('Raw Odom ω', '#38bdf8'), ds('Raw IMU ω', '#fbbf24'), ds('EKF Filtered ω', '#4ade80')] },
  options: chartOpts('rad/s'),
});

const chartEkfV = new Chart(document.getElementById('chartEkfV'), {
  type: 'line',
  data: { labels: [], datasets: [ds('Raw Odom v', '#38bdf8'), ds('EKF Filtered v', '#4ade80')] },
  options: chartOpts('m/s'),
});

function trim(chart) {
  while (chart.data.labels.length > MAX_POINTS) {
    chart.data.labels.shift();
    chart.data.datasets.forEach(d => d.data.shift());
  }
}

async function pollPID() {
  try {
    const resp = await fetch('/api/pid?since=' + lastPidId);
    const points = await resp.json();

    if (points.length > 0) {
      document.getElementById('statusDot').classList.add('active');
      document.getElementById('statusText').textContent = 'Streaming Live';

      for (const p of points) {
        lastPidId = p.id;
        const tLabel = p.t.toFixed(1);

        chartPid1.data.labels.push(tLabel);
        chartPid1.data.datasets[0].data.push(p.error);
        chartPid1.data.datasets[1].data.push(p.output);

        chartPid2.data.labels.push(tLabel);
        chartPid2.data.datasets[0].data.push(p.p);
        chartPid2.data.datasets[1].data.push(p.i);
        chartPid2.data.datasets[2].data.push(p.d);
      }

      trim(chartPid1); trim(chartPid2);
      chartPid1.update(); chartPid2.update();

      const lp = points[points.length - 1];
      document.getElementById('valMAE').textContent    = lp.mae.toFixed(5);
      document.getElementById('valError').textContent  = lp.error.toFixed(4);
      document.getElementById('valOutput').textContent = lp.output.toFixed(4);
      document.getElementById('valTarget').textContent = lp.target_yaw.toFixed(3);
      document.getElementById('valCurrent').textContent= lp.current_yaw.toFixed(3);
      document.getElementById('valP').textContent      = lp.p.toFixed(4);
      document.getElementById('valI').textContent      = lp.i.toFixed(4);
      document.getElementById('valD').textContent      = lp.d.toFixed(4);
      document.getElementById('valHold').innerHTML = '<span class="heading-hold on">ON</span>';
    }
  } catch (e) {}
}

async function pollEKF() {
  try {
    const resp = await fetch('/api/ekf?since=' + lastEkfId);
    const points = await resp.json();

    if (points.length > 0) {
      for (const p of points) {
        lastEkfId = p.id;
        const tLabel = p.t.toFixed(1);

        chartEkfYaw.data.labels.push(tLabel);
        chartEkfYaw.data.datasets[0].data.push(p.odom_yaw);
        chartEkfYaw.data.datasets[1].data.push(p.imu_yaw);
        chartEkfYaw.data.datasets[2].data.push(p.ekf_yaw);

        chartEkfW.data.labels.push(tLabel);
        chartEkfW.data.datasets[0].data.push(p.odom_w);
        chartEkfW.data.datasets[1].data.push(p.imu_w);
        chartEkfW.data.datasets[2].data.push(p.ekf_w);

        chartEkfV.data.labels.push(tLabel);
        chartEkfV.data.datasets[0].data.push(p.odom_v);
        chartEkfV.data.datasets[1].data.push(p.ekf_v);
      }

      trim(chartEkfYaw); trim(chartEkfW); trim(chartEkfV);
      chartEkfYaw.update(); chartEkfW.update(); chartEkfV.update();

      const lp = points[points.length - 1];
      document.getElementById('valOdomYaw').textContent = lp.odom_yaw.toFixed(3);
      document.getElementById('valImuYaw').textContent  = lp.imu_yaw.toFixed(3);
      document.getElementById('valEkfYaw').textContent  = lp.ekf_yaw.toFixed(3);
      document.getElementById('valImuW').textContent    = lp.imu_w.toFixed(3);
      document.getElementById('valEkfW').textContent    = lp.ekf_w.toFixed(3);
    }
  } catch (e) {}
}

async function savePIDValues() {
  try {
    const resp = await fetch('/api/save_params', { method: 'POST' });
    const res = await resp.json();
    alert(res.message);
  } catch (e) {
    alert('Failed to save parameters: ' + e);
  }
}

setInterval(() => { pollPID(); pollEKF(); }, 100);
</script>
</body>
</html>"""


class PIDDashboardNode(Node):
    """ROS 2 node that collects PID and EKF topics for web dashboard."""

    def __init__(self):
        super().__init__('pid_dashboard')
        global _start_time, _node_reference
        _start_time = time.time()
        _node_reference = self

        self.declare_parameter('port', 8080)
        self.port = self.get_parameter('port').value

        # Subscribers
        self.create_subscription(Float32MultiArray, '/pid_debug', self.pid_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_cb, 10)

        self.get_logger().info(f'Dashboard server starting on http://0.0.0.0:{self.port}')

    def pid_cb(self, msg):
        global _pid_point_id
        if len(msg.data) < 7:
            return
        t = time.time() - _start_time
        err = msg.data[0]

        _recent_errors.append(abs(err))
        mae = sum(_recent_errors) / len(_recent_errors) if _recent_errors else 0.0

        point = {
            'id': _pid_point_id,
            't': round(t, 3),
            'error': round(err, 6),
            'mae': round(mae, 6),
            'p': round(msg.data[1], 6),
            'i': round(msg.data[2], 6),
            'd': round(msg.data[3], 6),
            'output': round(msg.data[4], 6),
            'target_yaw': round(msg.data[5], 4),
            'current_yaw': round(msg.data[6], 4),
        }
        with _lock:
            _pid_data_buffer.append(point)
            _pid_point_id += 1

    def odom_cb(self, msg: Odometry):
        global _latest_odom_yaw, _latest_odom_v, _latest_odom_w
        _latest_odom_yaw = euler_from_quaternion(msg.pose.pose.orientation)
        _latest_odom_v = msg.twist.twist.linear.x
        _latest_odom_w = msg.twist.twist.angular.z

    def imu_cb(self, msg: Imu):
        global _latest_imu_yaw, _latest_imu_w
        _latest_imu_yaw = euler_from_quaternion(msg.orientation)
        _latest_imu_w = msg.angular_velocity.z

    def ekf_cb(self, msg: Odometry):
        global _ekf_point_id
        t = time.time() - _start_time
        ekf_yaw = euler_from_quaternion(msg.pose.pose.orientation)

        point = {
            'id': _ekf_point_id,
            't': round(t, 3),
            'odom_yaw': round(_latest_odom_yaw, 4),
            'imu_yaw': round(_latest_imu_yaw, 4),
            'ekf_yaw': round(ekf_yaw, 4),
            'odom_w': round(_latest_odom_w, 4),
            'imu_w': round(_latest_imu_w, 4),
            'ekf_w': round(msg.twist.twist.angular.z, 4),
            'odom_v': round(_latest_odom_v, 4),
            'ekf_v': round(msg.twist.twist.linear.x, 4),
        }
        with _lock:
            _ekf_data_buffer.append(point)
            _ekf_point_id += 1


class DashboardHandler(BaseHTTPRequestHandler):
    """HTTP Request Handler serving dashboard, JSON data, and CSV exports."""

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path in ('/', '/index.html'):
            self._serve_html()
        elif path == '/api/pid':
            self._serve_json_buffer(_pid_data_buffer, parsed.query)
        elif path == '/api/ekf':
            self._serve_json_buffer(_ekf_data_buffer, parsed.query)
        elif path == '/api/export_pid.csv':
            self._export_pid_csv()
        elif path == '/api/export_ekf.csv':
            self._export_ekf_csv()
        else:
            self.send_error(404)

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == '/api/save_params':
            self._handle_save_params()
        else:
            self.send_error(404)

    def _serve_html(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.end_headers()
        self.wfile.write(DASHBOARD_HTML.encode('utf-8'))

    def _serve_json_buffer(self, buffer, query_str):
        since_id = -1
        if query_str:
            qs = parse_qs(query_str)
            if 'since' in qs:
                try:
                    since_id = int(qs['since'][0])
                except ValueError:
                    pass

        with _lock:
            points = [p for p in buffer if p['id'] > since_id]

        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(json.dumps(points).encode('utf-8'))

    def _export_pid_csv(self):
        with _lock:
            points = list(_pid_data_buffer)

        csv_lines = ['timestamp_sec,yaw_error_rad,mae_rad,p_term,i_term,d_term,output_omega_rad_s,target_yaw_rad,current_yaw_rad']
        for p in points:
            csv_lines.append(f"{p['t']},{p['error']},{p['mae']},{p['p']},{p['i']},{p['d']},{p['output']},{p['target_yaw']},{p['current_yaw']}")

        csv_data = '\n'.join(csv_lines)

        self.send_response(200)
        self.send_header('Content-Type', 'text/csv')
        self.send_header('Content-Disposition', 'attachment; filename="pid_telemetry.csv"')
        self.end_headers()
        self.wfile.write(csv_data.encode('utf-8'))

    def _export_ekf_csv(self):
        with _lock:
            points = list(_ekf_data_buffer)

        csv_lines = ['timestamp_sec,raw_odom_yaw,raw_imu_yaw,ekf_filtered_yaw,raw_odom_w,raw_imu_w,ekf_filtered_w,raw_odom_v,ekf_filtered_v']
        for p in points:
            csv_lines.append(f"{p['t']},{p['odom_yaw']},{p['imu_yaw']},{p['ekf_yaw']},{p['odom_w']},{p['imu_w']},{p['ekf_w']},{p['odom_v']},{p['ekf_v']}")

        csv_data = '\n'.join(csv_lines)

        self.send_response(200)
        self.send_header('Content-Type', 'text/csv')
        self.send_header('Content-Disposition', 'attachment; filename="ekf_sensor_telemetry.csv"')
        self.end_headers()
        self.wfile.write(csv_data.encode('utf-8'))

    def _handle_save_params(self):
        yaml_path = os.path.expanduser('~/ros2_ws/src/straight_line_assistant/config/params.yaml')
        try:
            content = """straight_line_assistant:
  ros__parameters:
    kp: 1.5
    ki: 0.01
    kd: 0.2
    integral_limit: 1.0
    angular_epsilon: 0.001
    key_timeout: 0.6
    odom_timeout: 0.5
    control_rate: 20.0
    odom_topic: "/odometry/filtered"
"""
            os.makedirs(os.path.dirname(yaml_path), exist_ok=True)
            with open(yaml_path, 'w') as f:
                f.write(content)
            res = {'status': 'success', 'message': f'Saved PID parameters to {yaml_path}'}
        except Exception as e:
            res = {'status': 'error', 'message': f'Error saving params: {str(e)}'}

        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(res).encode('utf-8'))

    def log_message(self, format, *args):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PIDDashboardNode()

    server = HTTPServer(('0.0.0.0', node.port), DashboardHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    node.get_logger().info(f'Dashboard ready at http://0.0.0.0:{node.port}')

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
