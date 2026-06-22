import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Slider } from '@/components/ui/slider';
import { Label } from '@/components/ui/label';
import { type JointMeta } from '@/api';

const toDeg = (rad: number) => (rad * 180) / Math.PI;

interface Props {
  meta: JointMeta[];
  joints: number[];
  onChange: (index: number, value: number) => void;
  running: boolean;
}

/** 관절 각도 슬라이더 (관절공간 jog — 브라우저 FK, 백엔드 왕복 0). */
export default function JointPanel({ meta, joints, onChange, running }: Props) {
  if (meta.length === 0) return null;
  return (
    <Card className='bg-card/95 supports-[backdrop-filter]:bg-card/80 w-60 backdrop-blur'>
      <CardHeader className='pb-3'>
        <CardTitle className='text-sm'>관절 각도</CardTitle>
      </CardHeader>
      <CardContent className='space-y-4'>
        {meta.map((m, i) => (
          <div key={m.name} className='space-y-1.5'>
            <div className='flex items-center justify-between'>
              <Label className='text-muted-foreground text-xs'>{m.name}</Label>
              <span className='text-xs tabular-nums'>
                {toDeg(joints[i] ?? 0).toFixed(0)}°
              </span>
            </div>
            <Slider
              min={m.lower}
              max={m.upper}
              step={0.01}
              value={[joints[i] ?? 0]}
              disabled={running}
              onValueChange={([v]) => onChange(i, v)}
            />
          </div>
        ))}
      </CardContent>
    </Card>
  );
}
