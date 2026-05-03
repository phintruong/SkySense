import { useGNCStore } from "../hooks/useGNCStore";

const radToDeg = (value: number) => (value * 180) / Math.PI;

function Readout({
  label,
  value,
  unit = "",
  warn = false,
}: {
  label: string;
  value: string;
  unit?: string;
  warn?: boolean;
}) {
  return (
    <div className="flex items-center justify-between gap-4">
      <span className="text-gray-500">{label}</span>
      <span className={`font-mono ${warn ? "text-red-300" : "text-gray-100"}`}>
        {value}
        {unit && value !== "--" ? <span className="text-gray-500"> {unit}</span> : null}
      </span>
    </div>
  );
}

function Section({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <div>
      <p className="mb-2 text-[11px] font-semibold uppercase tracking-wider text-gray-500">
        {title}
      </p>
      <div className="space-y-1.5 text-sm">{children}</div>
    </div>
  );
}

export function TelemetryPanel() {
  const telemetry = useGNCStore((state) => state.telemetry);
  const targetAltitude = useGNCStore((state) => state.targetAltitude);

  const estimated = telemetry?.estimated_state;
  const eulerDeg = estimated?.euler.map(radToDeg) as [number, number, number] | undefined;
  const altitude = estimated ? -estimated.position[2] : null;
  const altitudeWarn = altitude !== null && Math.abs(altitude - targetAltitude) > 1;

  return (
    <section className="space-y-5">
      <Section title="Orientation">
        <Readout
          label="Roll"
          unit="deg"
          value={eulerDeg ? eulerDeg[0].toFixed(1) : "--"}
          warn={eulerDeg ? Math.abs(eulerDeg[0]) > 30 : false}
        />
        <Readout
          label="Pitch"
          unit="deg"
          value={eulerDeg ? eulerDeg[1].toFixed(1) : "--"}
          warn={eulerDeg ? Math.abs(eulerDeg[1]) > 30 : false}
        />
        <Readout label="Yaw" unit="deg" value={eulerDeg ? eulerDeg[2].toFixed(1) : "--"} />
      </Section>

      <Section title="Position">
        <Readout label="North" unit="m" value={estimated ? estimated.position[0].toFixed(2) : "--"} />
        <Readout label="East" unit="m" value={estimated ? estimated.position[1].toFixed(2) : "--"} />
        <Readout
          label="Alt"
          unit="m"
          value={altitude !== null ? altitude.toFixed(2) : "--"}
          warn={altitudeWarn}
        />
      </Section>

      <Section title="Velocity">
        <Readout label="Vn" value={estimated ? estimated.velocity[0].toFixed(2) : "--"} />
        <Readout label="Ve" value={estimated ? estimated.velocity[1].toFixed(2) : "--"} />
        <Readout label="Vd" value={estimated ? estimated.velocity[2].toFixed(2) : "--"} />
      </Section>

      <Section title="Gyro Bias">
        <Readout label="Bx" value={estimated ? estimated.gyro_bias[0].toFixed(4) : "--"} />
        <Readout label="By" value={estimated ? estimated.gyro_bias[1].toFixed(4) : "--"} />
        <Readout label="Bz" value={estimated ? estimated.gyro_bias[2].toFixed(4) : "--"} />
      </Section>
    </section>
  );
}
