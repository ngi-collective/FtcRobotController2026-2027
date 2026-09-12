// PROTOTYPE FIXTURE DATA — mirrors the wire-protocol message shapes decided in
// "Design: uniform dashboard wire protocol and backend adapter interface"
// (namespaces: opmode / telemetry / gamepad / device). Read-only, static —
// this prototype is answering "what should the UI look like", not "does the
// backend work", so all interactions below mutate local component state only.

export type OpModeStatus = 'stopped' | 'init' | 'running';

export interface OpModeSummary {
  name: string;
  flavor: 'teleop' | 'autonomous';
}

export const opModes: OpModeSummary[] = [
  { name: 'Basic Mecanum', flavor: 'teleop' },
  { name: 'Field Relative Mecanum (Sample)', flavor: 'teleop' },
  { name: 'I AM VERITY V2', flavor: 'teleop' },
  { name: 'Example Auto', flavor: 'autonomous' },
];

export interface TelemetryEntry {
  caption: string;
  value: string;
  timestamp: number;
}

export const initialTelemetry: TelemetryEntry[] = [
  { caption: 'status', value: 'initialized', timestamp: Date.now() - 4000 },
  { caption: 'heading', value: '12.4 deg', timestamp: Date.now() - 3000 },
  { caption: 'FL power', value: '0.65', timestamp: Date.now() - 2000 },
  { caption: 'FR power', value: '0.65', timestamp: Date.now() - 2000 },
  { caption: 'loop time', value: '18 ms', timestamp: Date.now() - 1000 },
];

export type BehaviorType = 'rampingToVelocity' | 'instant' | 'drifting';

export interface MotorDeviceState {
  kind: 'motor';
  name: string;
  commandedPower: number;
  currentPositionTicks: number;
  velocityTicksPerSec: number;
  behavior: { type: BehaviorType; maxTicksPerSec?: number; rampTimeMs?: number };
}

export interface ImuDeviceState {
  kind: 'imu';
  name: string;
  yawDeg: number;
  behavior: { type: BehaviorType; degreesPerSecond?: number };
}

export interface SensorDeviceState {
  kind: 'sensor';
  name: string;
  sensorType: 'touch' | 'color';
  value: boolean | string;
}

export type DeviceState = MotorDeviceState | ImuDeviceState | SensorDeviceState;

export const initialDevices: DeviceState[] = [
  {
    kind: 'motor',
    name: 'FL',
    commandedPower: 0.65,
    currentPositionTicks: 1240,
    velocityTicksPerSec: 1820,
    behavior: { type: 'rampingToVelocity', maxTicksPerSec: 2800, rampTimeMs: 200 },
  },
  {
    kind: 'motor',
    name: 'FR',
    commandedPower: 0.65,
    currentPositionTicks: 1198,
    velocityTicksPerSec: 1795,
    behavior: { type: 'rampingToVelocity', maxTicksPerSec: 2800, rampTimeMs: 200 },
  },
  {
    kind: 'motor',
    name: 'BL',
    commandedPower: 0.4,
    currentPositionTicks: 980,
    velocityTicksPerSec: 1100,
    behavior: { type: 'rampingToVelocity', maxTicksPerSec: 2800, rampTimeMs: 200 },
  },
  {
    kind: 'motor',
    name: 'BR',
    commandedPower: 0.4,
    currentPositionTicks: 966,
    velocityTicksPerSec: 1088,
    behavior: { type: 'rampingToVelocity', maxTicksPerSec: 2800, rampTimeMs: 200 },
  },
  { kind: 'imu', name: 'imu', yawDeg: 12.4, behavior: { type: 'drifting', degreesPerSecond: 0.5 } },
  { kind: 'sensor', name: 'touch1', sensorType: 'touch', value: false },
];

export interface GamepadState {
  left_stick_x: number;
  left_stick_y: number;
  right_stick_x: number;
  right_stick_y: number;
  a: boolean;
  b: boolean;
  x: boolean;
  y: boolean;
  left_bumper: boolean;
  right_bumper: boolean;
}

export const initialGamepad: GamepadState = {
  left_stick_x: 0,
  left_stick_y: 0,
  right_stick_x: 0,
  right_stick_y: 0,
  a: false,
  b: false,
  x: false,
  y: false,
  left_bumper: false,
  right_bumper: false,
};
