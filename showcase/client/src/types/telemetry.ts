export interface Vec3 {
  0: number;
  1: number;
  2: number;
  length: 3;
}

export interface TelemetryMessage {
  type: "telemetry";
  timestamp: number;
  true_state: {
    position: [number, number, number];
    velocity: [number, number, number];
    quaternion: [number, number, number, number]; // [qw, qx, qy, qz]
    euler: [number, number, number]; // [roll, pitch, yaw] radians
    angular_velocity: [number, number, number];
  };
  estimated_state: {
    position: [number, number, number];
    velocity: [number, number, number];
    quaternion: [number, number, number, number];
    euler: [number, number, number];
    gyro_bias: [number, number, number];
  };
  control: {
    thrust: number;
    torques: [number, number, number];
    motor_commands: [number, number, number, number];
    motor_actual: [number, number, number, number];
  };
  sensors: {
    imu_healthy: boolean;
    ultrasonic_healthy: boolean;
  };
  ekf: {
    innovation_norm: number;
    covariance_trace: number;
  };
  emergency: boolean;
}

export type SystemStatus = "NOMINAL" | "DEGRADED" | "DRIFTING" | "EMERGENCY";

export type SensorName = "imu" | "accel" | "ultrasonic";
export type FailureMode = "off" | "noisy" | "recover";

export interface DisturbanceCommand {
  type: "inject_disturbance";
  force: [number, number, number];
  duration: number;
}

export interface SensorFailureCommand {
  type: "sensor_failure";
  sensor: SensorName;
  mode: FailureMode;
}

export interface ResetCommand {
  type: "reset_sim";
}

export interface SetTargetCommand {
  type: "set_target";
  position: [number, number, number];
}

export type Command =
  | DisturbanceCommand
  | SensorFailureCommand
  | ResetCommand
  | SetTargetCommand;

export interface TelemetrySnapshot {
  timestamp: number;
  roll: number; // degrees
  pitch: number;
  yaw: number;
  accelX: number;
  accelY: number;
  accelZ: number;
  altitude: number; // positive-up meters
  targetAltitude: number;
  motor1: number;
  motor2: number;
  motor3: number;
  motor4: number;
}
