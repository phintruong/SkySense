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
    title: 'Wheels',
    summary: 'The front and rear wheels provide locomotion. We tested every tire we had and tuned the drive for the red and green tracks.',
    parts: [
      { id: 'wheel-1', role: 'Front/right wheel. Primary motion component for robot movement.' },
      { id: 'wheel-2', role: 'Rear/left wheel. Works with wheel-1 for stable driving and turns.' },
    ],
  },
  {
    title: 'Base Plate',
    summary: 'The laser-cut base plate is the structural foundation. Everything mounts to it: wheels, Arduino, sensors, and power.',
    parts: [
      { id: 'plate-1', role: 'Base plate. Supports all components and keeps the chassis rigid.' },
    ],
  },
  {
    title: 'IR Sensors',
    summary: 'Infrared proximity sensors detect obstacles and line edges. We use them for obstacle avoidance on the red track and line following.',
    parts: [
      { id: 'sensor-1', role: 'Left IR sensor. Detects obstacles and reflectance for line following.' },
      { id: 'sensor-2', role: 'Right IR sensor. Paired with sensor-1 for symmetric detection.' },
    ],
  },
  {
    title: 'Ultrasonic Sensor',
    summary: 'The ultrasonic distance sensor measures the distance to obstacles ahead using sound waves. Critical for obstacle detection and avoidance on the red track.',
    parts: [
      { id: 'ultrasonic-sensor-1', role: 'Ultrasonic distance sensor. Continuously monitors distance ahead to detect obstacles.' },
    ],
  },
  {
    title: 'Color Sensor',
    summary: 'The color/light sensor detects surface colors beneath the robot. Used for line following, detecting blue walls, and finding the black center in the bullseye challenge.',
    parts: [
      { id: 'color-sensor-1', role: 'Color/light sensor. Detects red lines, blue walls, black center, and other surface colors.' },
    ],
  },
  {
    title: 'Arduino Brain',
    summary: 'The Arduino Uno runs all our code: line following, obstacle avoidance, bullseye centering, and shooting logic.',
    parts: [
      { id: 'arduino-1', role: 'Main microcontroller. Reads sensors, drives motors, and runs the challenge algorithms.' },
    ],
  },
  {
    title: 'Motor Driver',
    summary: 'The L298N motor driver takes low-power signals from the Arduino and drives the wheel motors at the right speed and direction.',
    parts: [
      { id: 'motor-driver-1', role: 'H-bridge motor driver. Controls speed and direction of both wheels.' },
    ],
  },
  {
    title: 'Breadboard',
    summary: 'The breadboard lets us wire sensors and logic without soldering. Easy to change during testing.',
    parts: [
      { id: 'breadboard-1', role: 'Solderless prototyping board. Connects Arduino, sensors, and power cleanly.' },
    ],
  },
  {
    title: 'Power',
    summary: '67, 67 67 67 67 six seven SIX SEVEN',
    parts: [
      { id: 'battery-1', role: '67, 67 67 67 67 six seven SIX SEVEN' },
      { id: 'battery-2', role: '67, 67 67 67 67 six seven SIX SEVEN' },
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
