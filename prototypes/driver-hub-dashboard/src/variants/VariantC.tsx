import { useState } from 'react';
import type { VariantProps } from './types';

// Variant C — "Tabbed workspace": one concern fills the whole screen at a
// time. "Control" tab is a full-bleed virtual controller for manual driving
// sessions; "Telemetry" and "Devices" are dedicated full-screen views. No
// simultaneous panels — optimized for small screens / single-monitor focus,
// at the cost of not seeing telemetry while driving.
export function VariantC(props: VariantProps) {
  const [tab, setTab] = useState<'control' | 'telemetry' | 'devices'>('control');

  return (
    <div style={{ height: '100vh', display: 'flex', flexDirection: 'column', fontFamily: 'system-ui, sans-serif' }}>
      <div style={{ display: 'flex', borderBottom: '2px solid #ddd' }}>
        {(['control', 'telemetry', 'devices'] as const).map((t) => (
          <button
            key={t}
            onClick={() => setTab(t)}
            style={{
              flex: 1,
              padding: '14px 0',
              border: 'none',
              background: tab === t ? '#222' : '#f0f0f0',
              color: tab === t ? '#fff' : '#222',
              fontSize: 14,
              textTransform: 'uppercase',
              letterSpacing: 1,
              cursor: 'pointer',
            }}
          >
            {t}
          </button>
        ))}
      </div>

      <div style={{ flex: 1, overflow: 'auto' }}>
        {tab === 'control' && <ControlTab {...props} />}
        {tab === 'telemetry' && <TelemetryTab {...props} />}
        {tab === 'devices' && <DevicesTab {...props} />}
      </div>
    </div>
  );
}

function ControlTab({ opModes, selectedOpMode, status, onSelectOpMode, onInit, onStart, onStop, gamepad, onGamepadButton }: VariantProps) {
  return (
    <div style={{ height: '100%', display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'space-between', padding: 24, background: '#181818', color: '#fff' }}>
      <div style={{ display: 'flex', gap: 12, alignItems: 'center' }}>
        <select value={selectedOpMode} onChange={(e) => onSelectOpMode(e.target.value)} style={{ padding: 8 }}>
          {opModes.map((m) => (
            <option key={m.name} value={m.name}>
              {m.name}
            </option>
          ))}
        </select>
        <button onClick={onInit}>Init</button>
        <button onClick={onStart}>▶ Start</button>
        <button onClick={onStop}>■ Stop</button>
        <span>{status}</span>
      </div>

      <div style={{ display: 'flex', width: '100%', justifyContent: 'space-between', alignItems: 'center', padding: '0 40px' }}>
        <Stick label="left stick" x={gamepad.left_stick_x} y={gamepad.left_stick_y} />
        <div style={{ display: 'grid', gridTemplateColumns: 'repeat(2, 48px)', gap: 8 }}>
          {(['y', 'x', 'b', 'a'] as const).map((btn) => (
            <button
              key={btn}
              onClick={() => onGamepadButton(btn, !gamepad[btn])}
              style={{
                width: 48,
                height: 48,
                borderRadius: '50%',
                background: gamepad[btn] ? '#7CFC00' : '#333',
                color: gamepad[btn] ? '#111' : '#fff',
                border: 'none',
                fontWeight: 700,
              }}
            >
              {btn.toUpperCase()}
            </button>
          ))}
        </div>
        <Stick label="right stick" x={gamepad.right_stick_x} y={gamepad.right_stick_y} />
      </div>

      <div style={{ opacity: 0.5, fontSize: 12 }}>drag stick pads to simulate joystick input</div>
    </div>
  );
}

function Stick({ label, x, y }: { label: string; x: number; y: number }) {
  return (
    <div style={{ textAlign: 'center' }}>
      <div
        style={{
          width: 120,
          height: 120,
          borderRadius: '50%',
          background: '#222',
          border: '2px solid #444',
          position: 'relative',
        }}
      >
        <div
          style={{
            position: 'absolute',
            top: `${50 + y * 40}%`,
            left: `${50 + x * 40}%`,
            width: 24,
            height: 24,
            borderRadius: '50%',
            background: '#ff6b00',
            transform: 'translate(-50%, -50%)',
          }}
        />
      </div>
      <div style={{ fontSize: 11, marginTop: 6, opacity: 0.7 }}>{label}</div>
    </div>
  );
}

function TelemetryTab({ telemetry }: VariantProps) {
  return (
    <table style={{ width: '100%', borderCollapse: 'collapse' }}>
      <thead>
        <tr style={{ textAlign: 'left', borderBottom: '2px solid #ddd' }}>
          <th style={{ padding: 10 }}>time</th>
          <th style={{ padding: 10 }}>caption</th>
          <th style={{ padding: 10 }}>value</th>
        </tr>
      </thead>
      <tbody>
        {telemetry.map((t, i) => (
          <tr key={i} style={{ borderBottom: '1px solid #eee' }}>
            <td style={{ padding: 10, color: '#888' }}>{new Date(t.timestamp).toLocaleTimeString()}</td>
            <td style={{ padding: 10, fontWeight: 600 }}>{t.caption}</td>
            <td style={{ padding: 10 }}>{t.value}</td>
          </tr>
        ))}
      </tbody>
    </table>
  );
}

function DevicesTab({ devices, onOverrideMotorBehavior, onResetBehavior, onSetSensorValue }: VariantProps) {
  return (
    <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fill, minmax(260px, 1fr))', gap: 16, padding: 20 }}>
      {devices.map((d) => (
        <div key={d.name} style={{ border: '1px solid #ddd', borderRadius: 8, padding: 16 }}>
          <div style={{ fontWeight: 700, marginBottom: 10 }}>
            {d.name} <span style={{ fontWeight: 400, color: '#888' }}>{d.kind}</span>
          </div>
          {d.kind === 'motor' && (
            <>
              <div>power: {d.commandedPower.toFixed(2)}</div>
              <div>position: {d.currentPositionTicks} ticks</div>
              <div>velocity: {d.velocityTicksPerSec} tps</div>
              <label style={{ display: 'block', marginTop: 10, fontSize: 12 }}>
                override max velocity
                <input
                  type="number"
                  defaultValue={d.behavior.maxTicksPerSec}
                  onBlur={(e) => onOverrideMotorBehavior(d.name, Number(e.target.value), d.behavior.rampTimeMs ?? 200)}
                  style={{ display: 'block', width: '100%', marginTop: 4 }}
                />
              </label>
              <button onClick={() => onResetBehavior(d.name)} style={{ marginTop: 8 }}>
                reset
              </button>
            </>
          )}
          {d.kind === 'imu' && <div>yaw: {d.yawDeg.toFixed(1)}°</div>}
          {d.kind === 'sensor' && (
            <button onClick={() => onSetSensorValue(d.name, !(d.value as boolean))}>toggle ({String(d.value)})</button>
          )}
        </div>
      ))}
    </div>
  );
}
