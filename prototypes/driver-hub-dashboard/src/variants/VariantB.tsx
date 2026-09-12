import type { VariantProps } from './types';

// Variant B — "Cockpit": device-visualization-first. Left sidebar owns
// OpMode selection/lifecycle plus a device tree; the main area is a card
// grid of live gauges (one per device) with inline behavior editors above a
// telemetry feed; the virtual gamepad is a floating dock, like a real
// controller overlay. Optimized for "tune and watch simulated hardware" during
// an interactive session.
export function VariantB({
  opModes,
  selectedOpMode,
  status,
  onSelectOpMode,
  onInit,
  onStart,
  onStop,
  telemetry,
  devices,
  onOverrideMotorBehavior,
  onResetBehavior,
  onSetSensorValue,
  gamepad,
  onGamepadButton,
}: VariantProps) {
  return (
    <div style={{ display: 'flex', height: '100vh', background: '#f4f5f7', fontFamily: 'system-ui, sans-serif' }}>
      <div style={{ width: 260, background: '#1b2430', color: '#fff', padding: 16, overflowY: 'auto' }}>
        <h3 style={{ margin: '0 0 12px', fontSize: 15 }}>Driver Hub</h3>

        <div style={{ fontSize: 12, opacity: 0.7, marginBottom: 4 }}>OpMode</div>
        <select
          value={selectedOpMode}
          onChange={(e) => onSelectOpMode(e.target.value)}
          style={{ width: '100%', marginBottom: 8, padding: 6 }}
        >
          {opModes.map((m) => (
            <option key={m.name} value={m.name}>
              {m.name}
            </option>
          ))}
        </select>
        <div style={{ display: 'flex', gap: 6, marginBottom: 16 }}>
          <button onClick={onInit} style={sidebarBtn}>
            Init
          </button>
          <button onClick={onStart} style={{ ...sidebarBtn, background: '#2e7d32' }}>
            Start
          </button>
          <button onClick={onStop} style={{ ...sidebarBtn, background: '#c62828' }}>
            Stop
          </button>
        </div>
        <div style={{ fontSize: 12, marginBottom: 20 }}>
          status: <strong>{status}</strong>
        </div>

        <div style={{ fontSize: 12, opacity: 0.7, marginBottom: 4 }}>Devices</div>
        {devices.map((d) => (
          <div key={d.name} style={{ padding: '6px 8px', background: '#242f3f', borderRadius: 6, marginBottom: 6, fontSize: 13 }}>
            {d.name} <span style={{ opacity: 0.6 }}>({d.kind})</span>
          </div>
        ))}
      </div>

      <div style={{ flex: 1, overflowY: 'auto', padding: 20 }}>
        <h4 style={{ marginTop: 0 }}>Devices</h4>
        <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fill, minmax(220px, 1fr))', gap: 12, marginBottom: 24 }}>
          {devices.map((d) => (
            <div key={d.name} style={{ background: '#fff', borderRadius: 10, padding: 14, boxShadow: '0 1px 4px rgba(0,0,0,0.08)' }}>
              <div style={{ fontWeight: 600, marginBottom: 8 }}>{d.name}</div>
              {d.kind === 'motor' && (
                <>
                  <div style={{ height: 8, background: '#eee', borderRadius: 4, marginBottom: 6 }}>
                    <div
                      style={{
                        height: 8,
                        width: `${Math.min(100, (d.velocityTicksPerSec / (d.behavior.maxTicksPerSec ?? 2800)) * 100)}%`,
                        background: '#3f51b5',
                        borderRadius: 4,
                      }}
                    />
                  </div>
                  <div style={{ fontSize: 12, color: '#666' }}>vel {d.velocityTicksPerSec} tps · pwr {d.commandedPower.toFixed(2)}</div>
                  <label style={{ fontSize: 11, display: 'block', marginTop: 8 }}>
                    max ticks/sec
                    <input
                      type="range"
                      min={500}
                      max={4000}
                      value={d.behavior.maxTicksPerSec ?? 2800}
                      onChange={(e) => onOverrideMotorBehavior(d.name, Number(e.target.value), d.behavior.rampTimeMs ?? 200)}
                      style={{ width: '100%' }}
                    />
                  </label>
                  <button onClick={() => onResetBehavior(d.name)} style={resetLink}>
                    reset to default
                  </button>
                </>
              )}
              {d.kind === 'imu' && <div style={{ fontSize: 24, fontWeight: 700 }}>{d.yawDeg.toFixed(1)}°</div>}
              {d.kind === 'sensor' && (
                <button onClick={() => onSetSensorValue(d.name, !(d.value as boolean))} style={sidebarBtn}>
                  {d.value ? 'pressed' : 'released'} (toggle)
                </button>
              )}
            </div>
          ))}
        </div>

        <h4>Telemetry</h4>
        <div style={{ display: 'flex', flexDirection: 'column', gap: 6 }}>
          {telemetry.map((t, i) => (
            <div key={i} style={{ background: '#fff', borderRadius: 8, padding: '8px 12px', fontSize: 13, display: 'flex', justifyContent: 'space-between' }}>
              <span style={{ color: '#666' }}>{t.caption}</span>
              <strong>{t.value}</strong>
            </div>
          ))}
        </div>
      </div>

      <div
        style={{
          position: 'fixed',
          bottom: 20,
          right: 20,
          width: 160,
          height: 160,
          borderRadius: '50%',
          background: 'rgba(27,36,48,0.9)',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          color: '#fff',
          fontSize: 11,
          textAlign: 'center',
          boxShadow: '0 4px 16px rgba(0,0,0,0.3)',
        }}
      >
        <div>
          <div>gamepad1 dock</div>
          <div style={{ marginTop: 6 }}>
            LS ({gamepad.left_stick_x.toFixed(1)}, {gamepad.left_stick_y.toFixed(1)})
          </div>
          <button onClick={() => onGamepadButton('a', !gamepad.a)} style={{ marginTop: 6 }}>
            A: {gamepad.a ? 'on' : 'off'}
          </button>
        </div>
      </div>
    </div>
  );
}

const sidebarBtn: React.CSSProperties = {
  flex: 1,
  background: '#374863',
  color: '#fff',
  border: 'none',
  borderRadius: 6,
  padding: '6px 8px',
  cursor: 'pointer',
  fontSize: 12,
};

const resetLink: React.CSSProperties = {
  background: 'none',
  border: 'none',
  color: '#3f51b5',
  fontSize: 11,
  cursor: 'pointer',
  padding: 0,
  marginTop: 6,
};
