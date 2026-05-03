import { ConnectionBadge } from "./components/ConnectionBadge";
import { DroneViewer } from "./components/DroneViewer";
import { FailureControls } from "./components/FailureControls";
import { MotorBars } from "./components/MotorBars";
import { SystemStatus } from "./components/SystemStatus";
import { TelemetryGraphs } from "./components/TelemetryGraphs";
import { TelemetryPanel } from "./components/TelemetryPanel";
import { useTelemetrySocket } from "./hooks/useTelemetrySocket";

export default function App() {
  useTelemetrySocket();

  return (
    <div className="flex h-screen w-screen flex-col overflow-hidden bg-gray-950 text-gray-100">
      <div className="flex min-h-0 flex-1">
        <aside className="custom-scrollbar flex w-56 flex-col gap-6 overflow-y-auto border-r border-gray-800 bg-gray-950 px-4 py-4">
          <SystemStatus />
          <FailureControls />
        </aside>

        <main className="relative min-w-0 flex-1">
          <DroneViewer />
          <ConnectionBadge />
          <div className="pointer-events-none absolute left-4 top-4">
            <p className="text-[11px] font-semibold uppercase tracking-[0.24em] text-gray-500">
              SkySense GNC
            </p>
          </div>
        </main>

        <aside className="custom-scrollbar flex w-64 flex-col gap-6 overflow-y-auto border-l border-gray-800 bg-gray-950 px-4 py-4">
          <TelemetryPanel />
          <MotorBars />
        </aside>
      </div>
      <div className="h-48 flex-shrink-0 border-t border-gray-800 bg-gray-950">
        <TelemetryGraphs />
      </div>
    </div>
  );
}
