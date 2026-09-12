import type { DeviceState, GamepadState, OpModeStatus, OpModeSummary, TelemetryEntry } from '../mockData';

// Shared prop shape all three variants receive. Deliberately NOT a shared
// <Layout> — each variant owns its full page structure. Handlers are stubs
// (local state only): the question here is "what should this look like",
// not "does the backend work".
export interface VariantProps {
  opModes: OpModeSummary[];
  selectedOpMode: string;
  status: OpModeStatus;
  onSelectOpMode: (name: string) => void;
  onInit: () => void;
  onStart: () => void;
  onStop: () => void;

  telemetry: TelemetryEntry[];

  devices: DeviceState[];
  onOverrideMotorBehavior: (name: string, maxTicksPerSec: number, rampTimeMs: number) => void;
  onResetBehavior: (name: string) => void;
  onSetSensorValue: (name: string, value: boolean) => void;

  gamepad: GamepadState;
  onGamepadStick: (stick: 'left' | 'right', x: number, y: number) => void;
  onGamepadButton: (button: keyof GamepadState, pressed: boolean) => void;
}
