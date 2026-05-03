import { useGNCStore } from "../hooks/useGNCStore";
import type { SystemStatus as Status } from "../types/telemetry";

const statusClasses: Record<Status, string> = {
  NOMINAL: "border-emerald-500/50 bg-emerald-500/15 text-emerald-300",
  DEGRADED: "border-yellow-500/50 bg-yellow-500/15 text-yellow-300",
  DRIFTING: "border-orange-500/50 bg-orange-500/15 text-orange-300",
  EMERGENCY: "animate-pulse border-red-500/60 bg-red-500/20 text-red-300",
};

function HealthDot({ healthy }: { healthy: boolean }) {
  return (
    <span
      className={`h-2 w-2 rounded-full ${healthy ? "bg-emerald-400" : "bg-red-400"}`}
    />
  );
}

export function SystemStatus() {
  const status = useGNCStore((state) => state.status);
  const telemetry = useGNCStore((state) => state.telemetry);

  return (
    <section className="space-y-3">
      <div>
        <p className="text-[11px] font-semibold uppercase tracking-wider text-gray-500">
          System
        </p>
        <div
          className={`mt-2 rounded border px-3 py-3 text-center text-lg font-bold tracking-wide transition-colors duration-300 ${statusClasses[status]}`}
        >
          {status}
        </div>
      </div>

      <div className="space-y-2 text-sm">
        <div className="flex items-center justify-between gap-3">
          <span className="text-gray-400">Innovation</span>
          <span className="font-mono text-gray-100">
            {telemetry ? telemetry.ekf.innovation_norm.toFixed(1) : "--"}
          </span>
        </div>
        <div className="flex items-center justify-between gap-3">
          <span className="text-gray-400">Covariance</span>
          <span className="font-mono text-gray-100">
            {telemetry ? telemetry.ekf.covariance_trace.toFixed(2) : "--"}
          </span>
        </div>
        <div className="flex items-center justify-between gap-3">
          <span className="text-gray-400">IMU</span>
          <span className="flex items-center gap-2 font-mono text-gray-100">
            <HealthDot healthy={telemetry?.sensors.imu_healthy ?? false} />
            {telemetry ? (telemetry.sensors.imu_healthy ? "OK" : "FAIL") : "--"}
          </span>
        </div>
        <div className="flex items-center justify-between gap-3">
          <span className="text-gray-400">Ultrasonic</span>
          <span className="flex items-center gap-2 font-mono text-gray-100">
            <HealthDot healthy={telemetry?.sensors.ultrasonic_healthy ?? false} />
            {telemetry ? (telemetry.sensors.ultrasonic_healthy ? "OK" : "FAIL") : "--"}
          </span>
        </div>
      </div>
    </section>
  );
}
