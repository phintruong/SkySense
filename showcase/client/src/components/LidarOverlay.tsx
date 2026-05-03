/** 2D radar overlay: renders live LiDAR scan data on an HTML canvas. */
import { useRef, useEffect, useCallback } from 'react';
import { useRobotStore } from '../hooks/useRobotModel';

const RADAR_SIZE = 220;
const RADAR_PADDING = 16;
const MAX_RANGE = 3; // meters — matches RPLIDAR max range
const DOT_RADIUS = 2.5;

function distanceColor(d: number): string {
  if (d < 0.3) return '#ef4444'; // red — danger
  if (d < 0.5) return '#eab308'; // yellow — warning
  return '#22c55e'; // green — clear
}

const ACTION_LABELS: Record<string, string> = {
  MOVE_FORWARD: 'FORWARD',
  TURN_LEFT: 'TURN LEFT',
  TURN_RIGHT: 'TURN RIGHT',
  MOVE_BACKWARD: 'BACKWARD',
  HOVER: 'HOVER',
};

export default function LidarOverlay() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const lidar = useRobotStore((s) => s.lidar);

  const draw = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const size = RADAR_SIZE;
    const cx = size / 2;
    const cy = size / 2;
    const radius = size / 2 - 12;

    // Clear
    ctx.clearRect(0, 0, size, size);

    // Background circle
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(15, 23, 42, 0.85)';
    ctx.fill();

    // Range rings
    for (let i = 1; i <= 3; i++) {
      const r = (i / 3) * radius;
      ctx.beginPath();
      ctx.arc(cx, cy, r, 0, Math.PI * 2);
      ctx.strokeStyle = 'rgba(148, 163, 184, 0.25)';
      ctx.lineWidth = 0.5;
      ctx.stroke();
    }

    // Crosshairs
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.2)';
    ctx.lineWidth = 0.5;
    ctx.beginPath();
    ctx.moveTo(cx, cy - radius);
    ctx.lineTo(cx, cy + radius);
    ctx.moveTo(cx - radius, cy);
    ctx.lineTo(cx + radius, cy);
    ctx.stroke();

    // Forward cone (±80°)
    ctx.save();
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    const coneStart = -Math.PI / 2 - (80 * Math.PI) / 180;
    const coneEnd = -Math.PI / 2 + (80 * Math.PI) / 180;
    ctx.arc(cx, cy, radius, coneStart, coneEnd);
    ctx.closePath();
    ctx.fillStyle = 'rgba(56, 189, 248, 0.08)';
    ctx.fill();
    ctx.restore();

    // Scan points
    for (const pt of lidar.scan) {
      const angleDeg = pt.angle;
      // Rotate so 0° (forward) points up on the radar
      const angleRad = ((angleDeg - 90) * Math.PI) / 180;
      const dist = Math.min(pt.distance, MAX_RANGE);
      const r = (dist / MAX_RANGE) * radius;
      const px = cx + Math.cos(angleRad) * r;
      const py = cy + Math.sin(angleRad) * r;

      ctx.beginPath();
      ctx.arc(px, py, DOT_RADIUS, 0, Math.PI * 2);
      ctx.fillStyle = distanceColor(pt.distance);
      ctx.fill();
    }

    // Center dot (drone)
    ctx.beginPath();
    ctx.arc(cx, cy, 3, 0, Math.PI * 2);
    ctx.fillStyle = '#38bdf8';
    ctx.fill();

    // Forward indicator triangle
    ctx.beginPath();
    ctx.moveTo(cx, cy - radius - 6);
    ctx.lineTo(cx - 4, cy - radius);
    ctx.lineTo(cx + 4, cy - radius);
    ctx.closePath();
    ctx.fillStyle = '#38bdf8';
    ctx.fill();
  }, [lidar.scan]);

  useEffect(() => {
    draw();
  }, [draw]);

  const actionLabel = ACTION_LABELS[lidar.action] ?? lidar.action;
  const closest = lidar.obstacles.length
    ? Math.min(...lidar.obstacles.map((o) => o.distance)).toFixed(2)
    : null;

  return (
    <div
      className="fixed z-20 flex flex-col items-center gap-1"
      style={{ bottom: RADAR_PADDING, left: RADAR_PADDING }}
    >
      {/* Connection badge */}
      <div className="flex items-center gap-1.5 mb-1">
        <span
          className="inline-block w-2 h-2 rounded-full"
          style={{ backgroundColor: lidar.connected ? '#22c55e' : '#ef4444' }}
        />
        <span className="text-[10px] text-gray-400 uppercase tracking-wider">
          {lidar.connected ? 'Telemetry Live' : 'Disconnected'}
        </span>
      </div>

      {/* Radar canvas */}
      <canvas
        ref={canvasRef}
        width={RADAR_SIZE}
        height={RADAR_SIZE}
        className="rounded-full"
        style={{ imageRendering: 'auto' }}
      />

      {/* Action + stats */}
      {lidar.connected && lidar.action && (
        <div className="flex flex-col items-center mt-1">
          <span
            className="text-xs font-bold tracking-wider px-2 py-0.5 rounded"
            style={{
              color:
                lidar.action === 'MOVE_FORWARD'
                  ? '#22c55e'
                  : lidar.action === 'HOVER'
                    ? '#eab308'
                    : '#38bdf8',
              background: 'rgba(15,23,42,0.7)',
            }}
          >
            {actionLabel}
          </span>
          {closest && (
            <span className="text-[10px] text-gray-400 mt-0.5">
              {lidar.obstacles.length} obstacle{lidar.obstacles.length !== 1 ? 's' : ''} · nearest {closest} m
            </span>
          )}
        </div>
      )}
    </div>
  );
}
