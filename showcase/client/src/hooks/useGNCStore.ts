import { create } from "zustand";
import type {
  Command,
  SystemStatus,
  TelemetryMessage,
  TelemetrySnapshot,
} from "../types/telemetry";
import { classifyStatus } from "../utils/statusClassifier";

const MAX_HISTORY = 200;
const RAD_TO_DEG = 180 / Math.PI;

interface FailureState {
  ultrasonic: boolean;
  accel: boolean;
  imu: boolean;
}

interface GNCState {
  connected: boolean;
  setConnected: (v: boolean) => void;

  telemetry: TelemetryMessage | null;
  status: SystemStatus;
  updateTelemetry: (msg: TelemetryMessage) => void;

  history: TelemetrySnapshot[];

  activeFailures: FailureState;
  setFailureActive: (sensor: keyof FailureState, active: boolean) => void;

  targetAltitude: number;
  setTargetAltitude: (alt: number) => void;

  sendCommand: ((cmd: Command) => void) | null;
  setSendCommand: (fn: ((cmd: Command) => void) | null) => void;

  events: { timestamp: number; label: string }[];
  addEvent: (label: string) => void;
}

export const useGNCStore = create<GNCState>((set, get) => ({
  connected: false,
  setConnected: (v) => set({ connected: v }),

  telemetry: null,
  status: "NOMINAL",
  updateTelemetry: (msg) => {
    const est = msg.estimated_state;
    const [p, q, r] = msg.true_state.angular_velocity;
    const [motor1, motor2, motor3, motor4] = msg.control.motor_actual;
    const snapshot: TelemetrySnapshot = {
      timestamp: msg.timestamp,
      roll: est.euler[0] * RAD_TO_DEG,
      pitch: est.euler[1] * RAD_TO_DEG,
      yaw: est.euler[2] * RAD_TO_DEG,
      accelX: p,
      accelY: q,
      accelZ: r,
      altitude: -est.position[2],
      targetAltitude: get().targetAltitude,
      motor1,
      motor2,
      motor3,
      motor4,
    };

    set((state) => ({
      telemetry: msg,
      status: classifyStatus(msg),
      history: [...state.history, snapshot].slice(-MAX_HISTORY),
    }));
  },

  history: [],

  activeFailures: { ultrasonic: false, accel: false, imu: false },
  setFailureActive: (sensor, active) =>
    set((state) => ({
      activeFailures: { ...state.activeFailures, [sensor]: active },
    })),

  targetAltitude: 2.0,
  setTargetAltitude: (alt) => set({ targetAltitude: alt }),

  sendCommand: null,
  setSendCommand: (fn) => set({ sendCommand: fn }),

  events: [],
  addEvent: (label) => {
    const timestamp = get().telemetry?.timestamp ?? 0;
    set((state) => ({
      events: [...state.events.slice(-50), { timestamp, label }],
    }));
  },
}));
