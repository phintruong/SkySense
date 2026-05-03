import { useGNCStore } from "../hooks/useGNCStore";
import type { SensorName } from "../types/telemetry";

const baseButton =
  "w-full rounded border px-3 py-2 text-left text-sm font-medium transition-colors duration-300 disabled:cursor-not-allowed disabled:border-gray-800 disabled:bg-gray-900 disabled:text-gray-600";

export function FailureControls() {
  const connected = useGNCStore((state) => state.connected);
  const activeFailures = useGNCStore((state) => state.activeFailures);
  const targetAltitude = useGNCStore((state) => state.targetAltitude);
  const sendCommand = useGNCStore((state) => state.sendCommand);
  const setFailureActive = useGNCStore((state) => state.setFailureActive);
  const addEvent = useGNCStore((state) => state.addEvent);
  const setTargetAltitude = useGNCStore((state) => state.setTargetAltitude);

  const toggleSensor = (
    sensor: SensorName,
    active: boolean,
    failMode: "off" | "noisy",
    label: string,
  ) => {
    const nextActive = !active;
    sendCommand?.({
      type: "sensor_failure",
      sensor,
      mode: nextActive ? failMode : "recover",
    });
    setFailureActive(sensor, nextActive);
    addEvent(nextActive ? label : `Recover ${label}`);
  };

  const recoverAll = () => {
    (["ultrasonic", "accel", "imu"] as const).forEach((sensor) => {
      sendCommand?.({ type: "sensor_failure", sensor, mode: "recover" });
      setFailureActive(sensor, false);
    });
    addEvent("Recover All");
  };

  const resetSim = () => {
    sendCommand?.({ type: "reset_sim" });
    (["ultrasonic", "accel", "imu"] as const).forEach((sensor) =>
      setFailureActive(sensor, false),
    );
    addEvent("Reset Sim");
  };

  const updateAltitude = (value: number) => {
    setTargetAltitude(value);
    sendCommand?.({ type: "set_target", position: [0, 0, -value] });
  };

  return (
    <section className="space-y-4">
      <div>
        <p className="text-[11px] font-semibold uppercase tracking-wider text-gray-500">
          Failure Injection
        </p>
        <div className="mt-2 space-y-2">
          <button
            className={`${baseButton} border-cyan-500/40 bg-cyan-500/10 text-cyan-200 hover:bg-cyan-500/20`}
            disabled={!connected}
            onClick={() => {
              sendCommand?.({
                type: "inject_disturbance",
                force: [5, 0, 0],
                duration: 0.5,
              });
              addEvent("Wind Gust");
            }}
            type="button"
          >
            Wind Gust
          </button>
          <button
            className={`${baseButton} ${
              activeFailures.ultrasonic
                ? "border-red-500/60 bg-red-500/20 text-red-200"
                : "border-gray-800 bg-gray-900/70 text-gray-200 hover:bg-gray-800"
            }`}
            disabled={!connected}
            onClick={() =>
              toggleSensor(
                "ultrasonic",
                activeFailures.ultrasonic,
                "off",
                "Kill Ultrasonic",
              )
            }
            type="button"
          >
            Kill Ultrasonic
          </button>
          <button
            className={`${baseButton} ${
              activeFailures.accel
                ? "border-orange-500/60 bg-orange-500/20 text-orange-200"
                : "border-gray-800 bg-gray-900/70 text-gray-200 hover:bg-gray-800"
            }`}
            disabled={!connected}
            onClick={() =>
              toggleSensor("accel", activeFailures.accel, "noisy", "Degrade Accel")
            }
            type="button"
          >
            Degrade Accel
          </button>
          <button
            className={`${baseButton} ${
              activeFailures.imu
                ? "border-red-500/60 bg-red-500/20 text-red-200"
                : "border-gray-800 bg-gray-900/70 text-gray-200 hover:bg-gray-800"
            }`}
            disabled={!connected}
            onClick={() => toggleSensor("imu", activeFailures.imu, "off", "IMU Failure")}
            type="button"
          >
            IMU Failure
          </button>
          <button
            className={`${baseButton} border-emerald-500/40 bg-emerald-500/10 text-emerald-200 hover:bg-emerald-500/20`}
            disabled={!connected}
            onClick={recoverAll}
            type="button"
          >
            Recover All
          </button>
          <button
            className={`${baseButton} border-blue-500/40 bg-blue-500/10 text-blue-200 hover:bg-blue-500/20`}
            disabled={!connected}
            onClick={resetSim}
            type="button"
          >
            Reset Sim
          </button>
        </div>
      </div>

      <div>
        <div className="flex items-center justify-between">
          <p className="text-[11px] font-semibold uppercase tracking-wider text-gray-500">
            Target Altitude
          </p>
          <span className="font-mono text-sm text-gray-100">
            {targetAltitude.toFixed(1)} m
          </span>
        </div>
        <input
          aria-label="Target altitude"
          className="mt-3 h-2 w-full accent-cyan-400 disabled:opacity-40"
          disabled={!connected}
          max={4}
          min={0.5}
          onChange={(event) => updateAltitude(Number(event.target.value))}
          step={0.1}
          type="range"
          value={targetAltitude}
        />
      </div>
    </section>
  );
}
