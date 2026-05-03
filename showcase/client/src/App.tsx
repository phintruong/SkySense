/** Main app: 3D robot, parts list, sidebar, and live telemetry connection. */
import RobotViewer from './components/RobotViewer';
import ControlPanel from './components/ControlPanel';
import PartInfoPanel from './components/PartInfoPanel';
import RightSidebar from './components/RightSidebar';
import LidarOverlay from './components/LidarOverlay';
import { useTelemetrySocket } from './hooks/useLidarSocket';

export default function App() {
  useTelemetrySocket();

  return (
    <div className="w-full h-screen bg-gray-100 relative overflow-hidden">
      <RobotViewer />
      <ControlPanel />
      <PartInfoPanel />
      <RightSidebar />
      <LidarOverlay />

      {/* Title */}
      <div className="fixed top-4 left-1/2 -translate-x-1/2 z-10 pointer-events-none">
        <h1 className="text-gray-400 text-sm font-medium tracking-wider uppercase">
          SkySense
        </h1>
      </div>
    </div>
  );
}
