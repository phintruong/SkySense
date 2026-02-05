#!/usr/bin/env python3
"""
Web-based LiDAR Visualization

Creates a simple HTTP server that displays real-time LIDAR data in a browser.
Access via http://<pi-ip>:8080
"""

import os
import sys
import json
import math
import threading
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler
from typing import List, Tuple, Optional

# Add parent directory to path for imports
_LOGIC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _LOGIC_DIR not in sys.path:
    sys.path.insert(0, _LOGIC_DIR)

from hardware.rplidar_reader import RPLidarReader

# Global variable to store latest scan data
latest_scan = []
scan_lock = threading.Lock()


class LidarHTTPHandler(SimpleHTTPRequestHandler):
    """HTTP handler that serves scan data as JSON."""

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(get_html().encode())
        elif self.path == '/scan':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            with scan_lock:
                self.wfile.write(json.dumps(latest_scan).encode())
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        # Suppress HTTP request logs
        pass


def get_html():
    """Return HTML page with Canvas-based visualization."""
    return """
<!DOCTYPE html>
<html>
<head>
    <title>RPLIDAR A1 - Live Scan</title>
    <style>
        body {
            margin: 0;
            padding: 20px;
            background: #000;
            color: #0f0;
            font-family: monospace;
            display: flex;
            flex-direction: column;
            align-items: center;
        }
        h1 {
            color: #0f0;
            text-shadow: 0 0 10px #0f0;
        }
        #info {
            margin: 10px 0;
            font-size: 14px;
        }
        canvas {
            border: 2px solid #0f0;
            box-shadow: 0 0 20px #0f0;
            background: #000;
        }
        .stats {
            margin-top: 10px;
            display: flex;
            gap: 30px;
        }
        .stat {
            display: flex;
            align-items: center;
            gap: 10px;
        }
        .color-box {
            width: 20px;
            height: 20px;
            border: 1px solid #0f0;
        }
    </style>
</head>
<body>
    <h1>🛸 RPLIDAR A1 - Live Scan</h1>
    <div id="info">Connecting to LIDAR...</div>
    <canvas id="radar" width="800" height="800"></canvas>
    <div class="stats">
        <div class="stat">
            <div class="color-box" style="background: #f00;"></div>
            <span>0-1m (Danger)</span>
        </div>
        <div class="stat">
            <div class="color-box" style="background: #ff0;"></div>
            <span>1-3m (Caution)</span>
        </div>
        <div class="stat">
            <div class="color-box" style="background: #0f0;"></div>
            <span>3-6m (Safe)</span>
        </div>
    </div>

    <script>
        const canvas = document.getElementById('radar');
        const ctx = canvas.getContext('2d');
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        const maxRadius = canvas.width / 2 - 20;
        const maxDistance = 6.0; // meters (6m for full room mapping)

        function drawReferenceCircles() {
            ctx.strokeStyle = '#004400';
            ctx.lineWidth = 1;

            // Draw circles every 1m
            for (let r = 1; r <= maxDistance; r += 1) {
                const radius = (r / maxDistance) * maxRadius;
                ctx.beginPath();
                ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
                ctx.stroke();

                // Label
                ctx.fillStyle = '#006600';
                ctx.font = '14px monospace';
                ctx.fillText(r + 'm', centerX + radius + 5, centerY);
            }

            // Draw crosshairs
            ctx.strokeStyle = '#003300';
            ctx.beginPath();
            ctx.moveTo(centerX, 0);
            ctx.lineTo(centerX, canvas.height);
            ctx.moveTo(0, centerY);
            ctx.lineTo(canvas.width, centerY);
            ctx.stroke();
        }

        function getColor(distance_m) {
            if (distance_m <= 1.0) return '#ff0000'; // Red (0-1m - danger zone)
            if (distance_m <= 3.0) return '#ffff00'; // Yellow (1-3m - caution)
            return '#00ff00'; // Green (3-6m - safe)
        }

        function drawScan(scanData) {
            // Clear canvas
            ctx.fillStyle = '#000';
            ctx.fillRect(0, 0, canvas.width, canvas.height);

            drawReferenceCircles();

            // Draw center dot
            ctx.fillStyle = '#0ff';
            ctx.beginPath();
            ctx.arc(centerX, centerY, 5, 0, 2 * Math.PI);
            ctx.fill();

            // Draw scan points
            scanData.forEach(point => {
                const [angle, distance] = point; // [angle_deg, distance_meters]
                if (distance < 0.001 || distance > maxDistance) return;

                const angleRad = angle * Math.PI / 180;
                const radius = (distance / maxDistance) * maxRadius;
                const x = centerX + radius * Math.cos(angleRad);
                const y = centerY - radius * Math.sin(angleRad);

                ctx.fillStyle = getColor(distance);
                ctx.beginPath();
                ctx.arc(x, y, 3, 0, 2 * Math.PI);
                ctx.fill();
            });

            // Update info
            document.getElementById('info').textContent =
                `Points: ${scanData.length} | FPS: ${fps.toFixed(1)}`;
        }

        let fps = 0;
        let lastTime = Date.now();
        let frameCount = 0;

        let firstFetch = true;
        function updateScan() {
            fetch('/scan')
                .then(response => response.json())
                .then(data => {
                    if (firstFetch) {
                        console.log('First scan received:', data.length, 'points');
                        if (data.length > 0) {
                            console.log('Sample point:', data[0]);
                        }
                        firstFetch = false;
                    }
                    drawScan(data);

                    // Calculate FPS
                    frameCount++;
                    const now = Date.now();
                    if (now - lastTime >= 1000) {
                        fps = frameCount / ((now - lastTime) / 1000);
                        frameCount = 0;
                        lastTime = now;
                    }
                })
                .catch(err => {
                    console.error('Error fetching scan:', err);
                    document.getElementById('info').textContent = 'Error: ' + err.message;
                });
        }

        // Update every 16ms (~60 FPS) for ultra-smooth visualization
        setInterval(updateScan, 16);
        updateScan();
    </script>
</body>
</html>
"""


def scan_thread(reader):
    """Background thread that continuously reads scan data."""
    global latest_scan

    print("Scan thread started, waiting for data...")
    scan_count = 0
    last_print_time = time.time()
    try:
        for scan_data in reader.iter_scans():
            scan_count += 1
            with scan_lock:
                latest_scan = scan_data

            # Print once every 2 seconds
            current_time = time.time()
            if current_time - last_print_time >= 2.0:
                print(f"\nScan #{scan_count}: {len(scan_data)} points")
                if scan_data:
                    # Show first 3 points as samples
                    for i, (angle, distance) in enumerate(scan_data[:3]):
                        print(f"  Point {i+1}: angle={angle:6.2f}°, distance={distance:.3f}m")
                    if len(scan_data) > 3:
                        print(f"  ... and {len(scan_data) - 3} more points")
                last_print_time = current_time
    except Exception as e:
        print(f"Scan thread error: {e}")
        import traceback
        traceback.print_exc()


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Web-based LIDAR visualization')
    parser.add_argument('port_device', nargs='?', default='/dev/ttyUSB0',
                        help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('motor_gpio', type=int, nargs='?', default=None,
                        help='GPIO pin for motor control (BCM numbering)')
    parser.add_argument('--http-port', type=int, default=8080,
                        help='HTTP server port (default: 8080)')

    args = parser.parse_args()

    print(f"RPLIDAR Web Visualizer")
    print(f"Port: {args.port_device}")
    if args.motor_gpio:
        print(f"Motor GPIO: {args.motor_gpio}")
    print(f"HTTP server: http://<your-pi-ip>:{args.http_port}")
    print()

    # Initialize LIDAR
    reader = RPLidarReader(port=args.port_device, motor_gpio_pin=args.motor_gpio)

    if not reader.connect():
        print("ERROR: Failed to connect to LIDAR")
        return 1

    if not reader.start():
        print("ERROR: Failed to start LIDAR")
        reader.disconnect()
        return 1

    print("LIDAR started successfully!")

    # Start scan thread
    thread = threading.Thread(target=scan_thread, args=(reader,), daemon=True)
    thread.start()

    # Start HTTP server
    server = HTTPServer(('0.0.0.0', args.http_port), LidarHTTPHandler)
    print(f"\n✓ Web server running on http://0.0.0.0:{args.http_port}")
    print(f"  Access from your browser: http://<raspberry-pi-ip>:{args.http_port}")
    print("\nPress Ctrl+C to stop\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        reader.disconnect()
        server.shutdown()
        print("Done!")

    return 0


if __name__ == "__main__":
    sys.exit(main())
