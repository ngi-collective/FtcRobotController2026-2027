// PROTOTYPE for "Prototype: web dashboard v1 UI"
// (ngi-collective/FtcRobotController2026-2027#17, part of wayfinder map #11).
//
// Plan: three structurally different variants of the Driver Hub dashboard's
// v1 view (OpMode control, telemetry, virtual gamepad, per-device
// visualizer/editor), switchable via ?variant=A|B|C, on this single
// throwaway route. All data is static fixture data matching the wire-
// protocol message shapes decided in "Design: uniform dashboard wire
// protocol and backend adapter interface" (#16); all mutations are local
// component state only — no real backend, per the prototype skill's
// "point mutations at a stub" rule.
//
// - A: "Console" — telemetry-first, dark scrolling log, narrow device rail.
// - B: "Cockpit" — sidebar + device-gauge grid + floating gamepad dock.
// - C: "Tabbed workspace" — one full-screen concern at a time (control/telemetry/devices).

import { useState } from 'react';
import { PrototypeSwitcher } from './PrototypeSwitcher';
import { VariantA } from './variants/VariantA';
import { VariantB } from './variants/VariantB';
import { VariantC } from './variants/VariantC';
import {
  initialDevices,
  initialGamepad,
  initialTelemetry,
  opModes,
  type DeviceState,
  type GamepadState,
  type OpModeStatus,
} from './mockData';

const VARIANTS = [
  { key: 'A', label: 'Console (telemetry-first)' },
  { key: 'B', label: 'Cockpit (device-first, sidebar)' },
  { key: 'C', label: 'Tabbed workspace (focus-first)' },
];

export default function App() {
  const [variant, setVariant] = useState(() => new URLSearchParams(window.location.search).get('variant') ?? 'A');

  const [selectedOpMode, setSelectedOpMode] = useState(opModes[0].name);
  const [status, setStatus] = useState<OpModeStatus>('stopped');
  const [telemetry] = useState(initialTelemetry);
  const [devices, setDevices] = useState<DeviceState[]>(initialDevices);
  const [gamepad, setGamepad] = useState<GamepadState>(initialGamepad);

  function handleVariantChange(key: string) {
    setVariant(key);
    const url = new URL(window.location.href);
    url.searchParams.set('variant', key);
    window.history.replaceState({}, '', url);
  }

  function overrideMotorBehavior(name: string, maxTicksPerSec: number, rampTimeMs: number) {
    setDevices((prev) =>
      prev.map((d) =>
        d.kind === 'motor' && d.name === name
          ? { ...d, behavior: { type: 'rampingToVelocity', maxTicksPerSec, rampTimeMs } }
          : d,
      ),
    );
  }

  function resetBehavior(name: string) {
    setDevices((prev) =>
      prev.map((d) =>
        d.name === name && d.kind === 'motor'
          ? { ...d, behavior: { type: 'rampingToVelocity', maxTicksPerSec: 2800, rampTimeMs: 200 } }
          : d,
      ),
    );
  }

  function setSensorValue(name: string, value: boolean) {
    setDevices((prev) => prev.map((d) => (d.kind === 'sensor' && d.name === name ? { ...d, value } : d)));
  }

  function gamepadStick(stick: 'left' | 'right', x: number, y: number) {
    setGamepad((g) => ({
      ...g,
      [`${stick}_stick_x`]: x,
      [`${stick}_stick_y`]: y,
    }));
  }

  function gamepadButton(button: keyof GamepadState, pressed: boolean) {
    setGamepad((g) => ({ ...g, [button]: pressed }));
  }

  const sharedProps = {
    opModes,
    selectedOpMode,
    status,
    onSelectOpMode: setSelectedOpMode,
    onInit: () => setStatus('init'),
    onStart: () => setStatus('running'),
    onStop: () => setStatus('stopped'),
    telemetry,
    devices,
    onOverrideMotorBehavior: overrideMotorBehavior,
    onResetBehavior: resetBehavior,
    onSetSensorValue: setSensorValue,
    gamepad,
    onGamepadStick: gamepadStick,
    onGamepadButton: gamepadButton,
  };

  return (
    <>
      {variant === 'A' && <VariantA {...sharedProps} />}
      {variant === 'B' && <VariantB {...sharedProps} />}
      {variant === 'C' && <VariantC {...sharedProps} />}
      <PrototypeSwitcher variants={VARIANTS} current={variant} onChange={handleVariantChange} />
    </>
  );
}
