import { useGNCStore } from "../hooks/useGNCStore";

const MAX_THRUST = 12;
const motorLabels = ["Front R", "Back R", "Back L", "Front L"];

function thrustColor(value: number) {
  const ratio = value / MAX_THRUST;
  if (ratio > 0.9) return "bg-red-400";
  if (ratio > 0.7) return "bg-orange-400";
  return "bg-teal-400";
}

export function MotorBars() {
  const telemetry = useGNCStore((state) => state.telemetry);
  const values = telemetry?.control.motor_actual ?? [0, 0, 0, 0];

  return (
    <section>
      <p className="mb-3 text-[11px] font-semibold uppercase tracking-wider text-gray-500">
        Motor Thrust
      </p>
      <div className="grid grid-cols-4 gap-3">
        {values.map((value, index) => {
          const fill = Math.max(0, Math.min(100, (value / MAX_THRUST) * 100));
          return (
            <div className="min-w-0 text-center" key={motorLabels[index]}>
              <div className="mb-2 h-28 rounded border border-gray-800 bg-gray-950 p-1">
                <div className="flex h-full items-end">
                  <div
                    className={`w-full rounded-sm transition-all duration-300 ${thrustColor(value)}`}
                    style={{ height: `${fill}%` }}
                  />
                </div>
              </div>
              <div className="truncate text-[10px] text-gray-500">{motorLabels[index]}</div>
              <div className="font-mono text-xs text-gray-100">{value.toFixed(1)}N</div>
            </div>
          );
        })}
      </div>
    </section>
  );
}
