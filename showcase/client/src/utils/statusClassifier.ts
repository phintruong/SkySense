import type { SystemStatus, TelemetryMessage } from "../types/telemetry";

export function classifyStatus(msg: TelemetryMessage): SystemStatus {
  if (msg.emergency) return "EMERGENCY";

  const posErr = Math.sqrt(
    msg.true_state.position.reduce(
      (sum, v, i) => sum + (v - msg.estimated_state.position[i]) ** 2,
      0,
    ),
  );
  if (posErr > 0.5) return "DRIFTING";

  if (!msg.sensors.imu_healthy || !msg.sensors.ultrasonic_healthy) {
    return "DEGRADED";
  }
  if (msg.ekf.innovation_norm > 2.0) return "DEGRADED";
  if (msg.ekf.covariance_trace > 1.0) return "DEGRADED";

  return "NOMINAL";
}
