import type { VariantProps } from './types';

// Variant A — "Console": telemetry-first. A dark scrolling monospace log
// dominates the screen; OpMode controls are a thin top bar; devices and
// gamepad are compressed into a narrow side rail. Optimized for "watch what
// the robot is doing right now" during an automated test run.
export function VariantA({
  opModes,
  selectedOpMode,
  status,
  onSelectOpMode,
  onInit,
  onStart,
  onStop,
  telemetry,
  devices,
  gamepad,
}: VariantProps) {
  return (
    <div style={{ display: 'flex', flexDirection: 'column', height: '100vh', background: '#0b0f14', color: '#d6f5d6' }}>
      <div
        style={{
          display: 'flex',
          alignItems: 'center',
          gap: 12,
          padding: '10px 16px',
          borderBottom: '1px solid #1e2a1e',
          fontFamily: 'monospace',
        }}
      >
        <strong style={{ color: '#7CFC00' }}>Driver Hub — Console</strong>
        <select
          value={selectedOpMode}
          onChange={(e) => onSelectOpMode(e.target.value)}
          style={selectStyle}
        >
          {opModes.map((m) => (
            <option key={m.name} value={m.name}>
              [{m.flavor}] {m.name}
            </option>
          ))}
        </select>
        <button onClick={onInit} style={btnStyle}>
          init
        </button>
        <button onClick={onStart} style={btnStyle}>
          start
        </button>
        <button onClick={onStop} style={btnStyle}>
          stop
        </button>
        <span style={{ marginLeft: 'auto', color: status === 'running' ? '#7CFC00' : '#888' }}>
          status: {status}
        </span>
      </div>

      <div style={{ flex: 1, display: 'flex', minHeight: 0 }}>
        <div
          style={{
            flex: 1,
            overflowY: 'auto',
            padding: 16,
            fontFamily: 'monospace',
            fontSize: 13,
            lineHeight: 1.6,
          }}
        >
          {telemetry.map((t, i) => (
            <div key={i}>
              <span style={{ color: '#555' }}>{new Date(t.timestamp).toLocaleTimeString()}</span>{' '}
              <span style={{ color: '#7CFC00' }}>{t.caption}</span>: {t.value}
            </div>
          ))}
          <div style={{ color: '#555' }}>▌ waiting for next telemetry.update()...</div>
        </div>

        <div
          style={{
            width: 220,
            borderLeft: '1px solid #1e2a1e',
            padding: 12,
            fontFamily: 'monospace',
            fontSize: 12,
            overflowY: 'auto',
          }}
        >
          <div style={{ color: '#7CFC00', marginBottom: 8 }}>devices</div>
          {devices.map((d) => (
            <div key={d.name} style={{ marginBottom: 6, color: '#9fd89f' }}>
              {d.kind === 'motor' && (
                <div>
                  {d.name}: pwr {d.commandedPower.toFixed(2)} vel {d.velocityTicksPerSec}
                </div>
              )}
              {d.kind === 'imu' && <div>{d.name}: yaw {d.yawDeg.toFixed(1)}°</div>}
              {d.kind === 'sensor' && (
                <div>
                  {d.name}: {String(d.value)}
                </div>
              )}
            </div>
          ))}

          <div style={{ color: '#7CFC00', margin: '16px 0 8px' }}>gamepad1</div>
          <div>
            LS ({gamepad.left_stick_x.toFixed(1)}, {gamepad.left_stick_y.toFixed(1)})
          </div>
          <div>
            RS ({gamepad.right_stick_x.toFixed(1)}, {gamepad.right_stick_y.toFixed(1)})
          </div>
        </div>
      </div>
    </div>
  );
}

const btnStyle: React.CSSProperties = {
  background: '#132313',
  color: '#7CFC00',
  border: '1px solid #2a4a2a',
  padding: '4px 10px',
  cursor: 'pointer',
  fontFamily: 'monospace',
};

const selectStyle: React.CSSProperties = {
  background: '#132313',
  color: '#d6f5d6',
  border: '1px solid #2a4a2a',
  padding: '4px 8px',
  fontFamily: 'monospace',
};
