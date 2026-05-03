import {
  CartesianGrid,
  Line,
  LineChart,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from "recharts";
import { useGNCStore } from "../hooks/useGNCStore";
import type { TelemetrySnapshot } from "../types/telemetry";

interface ChartSpec {
  title: string;
  yDomain?: [number, number];
  lines: { key: keyof TelemetrySnapshot; name: string; color: string; dashed?: boolean }[];
}

const charts: ChartSpec[] = [
  {
    title: "Attitude",
    yDomain: [-45, 45],
    lines: [
      { key: "roll", name: "Roll", color: "#f87171" },
      { key: "pitch", name: "Pitch", color: "#34d399" },
      { key: "yaw", name: "Yaw", color: "#60a5fa" },
    ],
  },
  {
    title: "Altitude",
    yDomain: [0, 5],
    lines: [
      { key: "altitude", name: "Est", color: "#22d3ee" },
      { key: "targetAltitude", name: "Target", color: "#a78bfa", dashed: true },
    ],
  },
  {
    title: "Angular Velocity",
    lines: [
      { key: "accelX", name: "p", color: "#fb7185" },
      { key: "accelY", name: "q", color: "#4ade80" },
      { key: "accelZ", name: "r", color: "#38bdf8" },
    ],
  },
  {
    title: "Motor Thrust",
    yDomain: [0, 12],
    lines: [
      { key: "motor1", name: "M1", color: "#2dd4bf" },
      { key: "motor2", name: "M2", color: "#facc15" },
      { key: "motor3", name: "M3", color: "#fb923c" },
      { key: "motor4", name: "M4", color: "#c084fc" },
    ],
  },
];

const formatTime = (value: number, data: TelemetrySnapshot[]) => {
  const first = data[0]?.timestamp ?? value;
  return `${Math.max(0, value - first).toFixed(0)}s`;
};

export function TelemetryGraphs() {
  const history = useGNCStore((state) => state.history);

  return (
    <div className="grid h-full grid-cols-4 gap-4 p-4">
      {charts.map((chart) => (
        <div className="min-w-0" key={chart.title}>
          <div className="mb-1 flex items-center justify-between">
            <p className="text-[11px] font-semibold uppercase tracking-wider text-gray-500">
              {chart.title}
            </p>
          </div>
          <ResponsiveContainer height={150} width="100%">
            <LineChart data={history} margin={{ bottom: 0, left: -18, right: 8, top: 8 }}>
              <CartesianGrid stroke="#1f2937" strokeDasharray="3 3" />
              <XAxis
                dataKey="timestamp"
                tick={{ fill: "#6b7280", fontSize: 10 }}
                tickFormatter={(value) => formatTime(Number(value), history)}
              />
              <YAxis
                domain={chart.yDomain}
                tick={{ fill: "#6b7280", fontSize: 10 }}
                width={36}
              />
              <Tooltip
                contentStyle={{
                  background: "#111827",
                  border: "1px solid #374151",
                  borderRadius: 4,
                  color: "#f3f4f6",
                }}
                labelFormatter={(value) => formatTime(Number(value), history)}
              />
              {chart.lines.map((line) => (
                <Line
                  dataKey={line.key}
                  dot={false}
                  isAnimationActive={false}
                  key={line.key}
                  name={line.name}
                  stroke={line.color}
                  strokeDasharray={line.dashed ? "5 4" : undefined}
                  strokeWidth={1.5}
                  type="monotone"
                />
              ))}
            </LineChart>
          </ResponsiveContainer>
        </div>
      ))}
    </div>
  );
}
