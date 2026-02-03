/** Right sidebar: camera, tour. */
import { useState, useEffect, useMemo, useRef } from 'react';
import { useRobotStore } from '../hooks/useRobotModel';

type TourStepPart = {
  id: string;
  role: string;
};

type TourStep = {
  title: string;
  summary: string;
  parts: TourStepPart[];
};

const TOUR_STEPS: TourStep[] = [
  {
    title: '360° LiDAR Sensor',
    summary: 'The LiDAR sensor is the "head" of the drone, providing 360-degree environmental scanning for navigation and obstacle detection.',
    parts: [
      { id: '360 LiDAR sensor-1', role: 'Primary navigation sensor. Scans the environment in all directions for mapping and obstacle avoidance.' },
    ],
  },
  {
    title: 'Ultrasonic Sensor',
    summary: 'The HC-SR04 ultrasonic sensor provides precise distance measurements for close-range obstacle detection.',
    parts: [
      { id: 'HCSR04 Ultrasonic Sensor-1', role: 'Distance sensor. Uses sound waves to measure proximity to nearby objects.' },
    ],
  },
  {
    title: 'Propulsion Motors',
    summary: 'Four brushless motors provide lift and thrust. Opposing motors spin in opposite directions for stability.',
    parts: [
      { id: 'crude motor-1', role: 'Motor #1. Generates thrust for lift and directional control.' },
      { id: 'crude motor-2', role: 'Motor #2. Works with motor #1 for balanced flight.' },
      { id: 'crude motor-3', role: 'Motor #3. Paired with motor #4 for stability.' },
      { id: 'crude motor-4', role: 'Motor #4. Completes the quadcopter configuration.' },
    ],
  },
  {
    title: 'Motor Arms',
    summary: 'The motor arms extend from the chassis to position the motors at optimal distance for stable flight.',
    parts: [
      { id: 'Motor Arm 1-1', role: 'Arm #1. Structural support for motor positioning.' },
      { id: 'Motor Arm 1-2', role: 'Arm #2. Maintains motor spacing for flight dynamics.' },
      { id: 'Motor Arm 1-3', role: 'Arm #3. Part of the X-configuration frame.' },
      { id: 'Motor Arm 1-4', role: 'Arm #4. Completes the quadcopter frame structure.' },
    ],
  },
  {
    title: 'Chassis Frame',
    summary: 'The two-plate chassis design houses all internal components while providing structural rigidity.',
    parts: [
      { id: 'Chassis - Bottom Plate-1', role: 'Bottom plate. Foundation for mounting components.' },
      { id: 'Chassis - Top Plate-2', role: 'Top plate. Protects components and mounts LiDAR sensor.' },
    ],
  },
  {
    title: 'Support Columns',
    summary: 'Four columns connect the top and bottom plates, creating space for internal electronics.',
    parts: [
      { id: 'Column-1', role: 'Structural spacer providing vertical support.' },
      { id: 'Column-2', role: 'Maintains plate separation and rigidity.' },
      { id: 'Column-3', role: 'Part of the four-corner support system.' },
      { id: 'Column-4', role: 'Completes the internal frame structure.' },
    ],
  },
  {
    title: 'Raspberry Pi 4',
    summary: 'The Raspberry Pi 4 is the brain of the drone, running Linux and flight control software.',
    parts: [
      { id: 'Raspberry Pi 4 Model B', role: 'Main computer. Processes sensor data and controls all drone functions.' },
      { id: 'Broadcom BCM2711B0 CPU, RPi4ModelB-1', role: 'Quad-core ARM processor running at 1.5GHz.' },
    ],
  },
  {
    title: 'Connectivity',
    summary: 'The Pi provides WiFi, Bluetooth, USB, and Ethernet for communication and peripherals.',
    parts: [
      { id: 'Cypress CYW43455 Wireless Module Cover, RPi4ModelB-1', role: 'WiFi and Bluetooth module for wireless telemetry.' },
      { id: 'Gigabit Ethernet Port, RPi4ModelB-1', role: 'Wired network for high-bandwidth data transfer.' },
      { id: '2X USB3.0 PORTS, RPi4ModelB-2', role: 'High-speed USB for cameras and storage.' },
    ],
  },
];

export default function RightSidebar() {
  const [tourOpen, setTourOpen] = useState(false);
  const [stepIndex, setStepIndex] = useState(0);

  // Store selectors
  const cameraMode = useRobotStore((s) => s.cameraMode);
  const toggleCameraMode = useRobotStore((s) => s.toggleCameraMode);
  const parts = useRobotStore((s) => s.parts);
  const highlightedParts = useRobotStore((s) => s.highlightedParts);
  const selectedPart = useRobotStore((s) => s.selectedPart);
  const highlightParts = useRobotStore((s) => s.highlightParts);
  const clearHighlights = useRobotStore((s) => s.clearHighlights);
  const selectPart = useRobotStore((s) => s.selectPart);
  const setTourActive = useRobotStore((s) => s.setTourActive);

  const previousSelectionRef = useRef<{
    highlightedParts: string[];
    selectedPart: string | null;
  } | null>(null);

  // Tour logic
  const clampedIndex = Math.min(Math.max(stepIndex, 0), TOUR_STEPS.length - 1);
  const step = TOUR_STEPS[clampedIndex];

  const resolvedParts = useMemo(() => {
    return step.parts.map((tourPart) => {
      const match = parts.find((part) => part.id === tourPart.id);
      return {
        ...tourPart,
        name: match?.name ?? tourPart.id,
        description: match?.description ?? null,
      };
    });
  }, [parts, step.parts]);

  useEffect(() => {
    if (!tourOpen) return;
    const ids = Array.from(new Set(step.parts.map((part) => part.id)));
    if (ids.length > 0) {
      highlightParts(ids);
    } else {
      clearHighlights();
    }
    selectPart(null);
  }, [tourOpen, step.parts, highlightParts, clearHighlights, selectPart]);

  const startTour = () => {
    previousSelectionRef.current = {
      highlightedParts: [...highlightedParts],
      selectedPart,
    };
    setTourActive(true);
    setStepIndex(0);
    setTourOpen(true);
  };

  const endTour = () => {
    setTourActive(false);
    setTourOpen(false);
    const previous = previousSelectionRef.current;
    if (previous) {
      highlightParts(previous.highlightedParts);
      selectPart(previous.selectedPart);
    } else {
      clearHighlights();
      selectPart(null);
    }
    previousSelectionRef.current = null;
  };

  const goPrevious = () => {
    setStepIndex((current) => Math.max(current - 1, 0));
  };

  const goNext = () => {
    if (clampedIndex >= TOUR_STEPS.length - 1) {
      endTour();
      return;
    }
    setStepIndex((current) => Math.min(current + 1, TOUR_STEPS.length - 1));
  };

  const toggleTour = () => {
    if (tourOpen) {
      endTour();
    } else {
      startTour();
    }
  };

  return (
    <div className="fixed top-4 right-4 z-20 w-64 right-sidebar-container">
      <div className="bg-white/90 backdrop-blur-md rounded-xl border border-gray-200 shadow-2xl overflow-hidden">
        {/* Camera Toggle */}
        <div className="px-3 py-2 border-b border-gray-200 flex items-center justify-between">
          <div className="flex items-center gap-2">
            <svg
              xmlns="http://www.w3.org/2000/svg"
              className="h-4 w-4 text-gray-500"
              fill="none"
              viewBox="0 0 24 24"
              stroke="currentColor"
            >
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={2}
                d="M15 10l4.553-2.276A1 1 0 0121 8.618v6.764a1 1 0 01-1.447.894L15 14M5 18h8a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z"
              />
            </svg>
            <span className="text-xs text-gray-700 font-medium">Camera</span>
          </div>
          <button
            onClick={toggleCameraMode}
            className="text-[10px] px-2 py-1 rounded bg-gray-100 text-gray-600 hover:bg-gray-200 transition-colors"
          >
            {cameraMode === 'third' ? 'Third Person' : 'First Person'}
          </button>
        </div>

        {/* Tour Guide Section */}
        <div>
          <button
            onClick={toggleTour}
            className="w-full px-3 py-2 flex items-center justify-between hover:bg-gray-50 transition-colors"
          >
            <div className="flex items-center gap-2">
              <svg
                xmlns="http://www.w3.org/2000/svg"
                className="h-4 w-4 text-gray-500"
                fill="none"
                viewBox="0 0 24 24"
                stroke="currentColor"
              >
                <path
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth={2}
                  d="M9 20l-5.447-2.724A1 1 0 013 16.382V5.618a1 1 0 011.447-.894L9 7m0 13l6-3m-6 3V7m6 10l4.553 2.276A1 1 0 0021 18.382V7.618a1 1 0 00-.553-.894L15 4m0 13V4m0 0L9 7"
                />
              </svg>
              <span className="text-xs text-gray-700 font-medium">Tour Guide</span>
            </div>
            <svg
              xmlns="http://www.w3.org/2000/svg"
              className={`h-4 w-4 text-gray-400 transition-transform ${tourOpen ? 'rotate-180' : ''}`}
              fill="none"
              viewBox="0 0 24 24"
              stroke="currentColor"
            >
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
            </svg>
          </button>

          {tourOpen && (
            <div className="px-3 pb-3">
              <div className="mb-2">
                <div className="text-[10px] text-gray-400 uppercase tracking-wide">Step {clampedIndex + 1} of {TOUR_STEPS.length}</div>
                <h3 className="text-gray-800 text-sm font-semibold">{step.title}</h3>
              </div>

              <p className="text-gray-600 text-xs leading-relaxed mb-2">{step.summary}</p>

              <div className="space-y-1 mb-3">
                <div className="text-[10px] text-gray-400 uppercase tracking-wide">Highlighted Parts</div>
                {resolvedParts.length === 0 ? (
                  <div className="text-[10px] text-gray-400">No parts assigned to this step.</div>
                ) : (
                  <div className="max-h-[200px] overflow-y-auto custom-scrollbar space-y-1">
                    {resolvedParts.map((part) => (
                      <div key={part.id} className="bg-gray-50 border border-gray-200 rounded-lg p-2">
                        <div className="flex items-center justify-between">
                          <div className="text-[10px] font-semibold text-gray-700">{part.name}</div>
                          <div className="text-[9px] text-gray-400">{part.id}</div>
                        </div>
                        <div className="text-[10px] text-gray-500">{part.role}</div>
                      </div>
                    ))}
                  </div>
                )}
              </div>

              <div className="flex items-center justify-between">
                <button
                  onClick={goPrevious}
                  disabled={clampedIndex === 0}
                  className="text-[10px] px-2 py-1 rounded border border-gray-200 text-gray-500 hover:text-gray-700 disabled:opacity-40 disabled:cursor-not-allowed"
                >
                  Back
                </button>
                <button
                  onClick={goNext}
                  className="text-[10px] px-2 py-1 rounded bg-blue-600 text-white hover:bg-blue-500 transition-colors"
                >
                  {clampedIndex >= TOUR_STEPS.length - 1 ? 'Finish' : 'Next'}
                </button>
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
