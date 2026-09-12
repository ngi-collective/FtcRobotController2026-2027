import { useEffect } from 'react';

export interface VariantMeta {
  key: string;
  label: string;
}

interface Props {
  variants: VariantMeta[];
  current: string;
  onChange: (key: string) => void;
}

// PROTOTYPE-ONLY UI. Never render in a production build.
export function PrototypeSwitcher({ variants, current, onChange }: Props) {
  const index = Math.max(
    0,
    variants.findIndex((v) => v.key === current),
  );

  function cycle(delta: number) {
    const next = (index + delta + variants.length) % variants.length;
    onChange(variants[next].key);
  }

  useEffect(() => {
    function handleKey(e: KeyboardEvent) {
      const target = e.target as HTMLElement | null;
      const tag = target?.tagName;
      if (tag === 'INPUT' || tag === 'TEXTAREA' || target?.isContentEditable) return;
      if (e.key === 'ArrowLeft') cycle(-1);
      if (e.key === 'ArrowRight') cycle(1);
    }
    window.addEventListener('keydown', handleKey);
    return () => window.removeEventListener('keydown', handleKey);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [index, variants]);

  const meta = variants[index];

  return (
    <div
      style={{
        position: 'fixed',
        bottom: 16,
        left: '50%',
        transform: 'translateX(-50%)',
        display: 'flex',
        alignItems: 'center',
        gap: 12,
        background: '#111',
        color: '#fff',
        padding: '8px 16px',
        borderRadius: 999,
        boxShadow: '0 4px 20px rgba(0,0,0,0.4)',
        fontFamily: 'monospace',
        fontSize: 13,
        zIndex: 9999,
        border: '2px solid #ff6b00',
      }}
    >
      <button onClick={() => cycle(-1)} style={arrowStyle}>
        ←
      </button>
      <span>
        PROTOTYPE — variant <strong>{meta.key}</strong> ({meta.label})
      </span>
      <button onClick={() => cycle(1)} style={arrowStyle}>
        →
      </button>
    </div>
  );
}

const arrowStyle: React.CSSProperties = {
  background: '#ff6b00',
  color: '#111',
  border: 'none',
  borderRadius: '50%',
  width: 24,
  height: 24,
  cursor: 'pointer',
  fontWeight: 'bold',
};
