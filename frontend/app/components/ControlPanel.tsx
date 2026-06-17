import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Slider } from '@/components/ui/slider';
import { Label } from '@/components/ui/label';
import { Button } from '@/components/ui/button';
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select';
import type { ControllerSpec, JointMeta } from '@/api';

interface Props {
  meta: JointMeta[];
  joints: number[];
  onChange: (index: number, value: number) => void;
  onReset: () => void;
  controllers: ControllerSpec[];
  controller: string;
  onControllerChange: (value: string) => void;
  gains: Record<string, number>;
  onGainChange: (key: string, value: number) => void;
  payload: number;
  onPayloadChange: (value: number) => void;
  modelScale: number;
  onModelScaleChange: (value: number) => void;
  onRun: () => void;
  running: boolean;
}

const toDeg = (rad: number) => (rad * 180) / Math.PI;

// 2차 PD 오차동역학 ë + Kd·ė + Kp·e = 0  →  ωn=√Kp, ζ=Kd/(2√Kp)
// (CT 는 비선형 상쇄 후 정확, PD/PID 는 M(θ) 변동으로 근사)
// Ki>0 이면 3차 s³+Kd·s²+Kp·s+Ki → Routh 안정조건 Ki<Kp·Kd, margin=Ki/(Kp·Kd)
function damping(kp?: number, kd?: number, ki?: number) {
  if (!kp || kp <= 0 || kd == null) return null;
  const wn = Math.sqrt(kp);
  const zeta = kd / (2 * wn);
  const label = zeta < 0.97 ? '과소감쇠' : zeta > 1.03 ? '과감쇠' : '임계감쇠';
  const color =
    zeta < 0.97
      ? 'text-amber-600'
      : zeta > 1.03
        ? 'text-blue-600'
        : 'text-emerald-600';
  const margin = ki && ki > 0 ? ki / (kp * kd) : null;
  return { wn, zeta, label, color, margin };
}

/** 관절 슬라이더 + 제어기/게인 튜닝 + Run 패널. */
export default function ControlPanel({
  meta,
  joints,
  onChange,
  onReset,
  controllers,
  controller,
  onControllerChange,
  gains,
  onGainChange,
  payload,
  onPayloadChange,
  modelScale,
  onModelScaleChange,
  onRun,
  running,
}: Props) {
  if (meta.length === 0) return null;
  const spec = controllers.find((c) => c.name === controller);
  const d = damping(gains.kp, gains.kd, gains.ki);

  return (
    <Card className='bg-card/95 supports-[backdrop-filter]:bg-card/80 w-72 backdrop-blur'>
      <CardHeader className='pb-3'>
        <CardTitle className='text-sm'>관절 제어</CardTitle>
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

        <div className='space-y-3 border-t pt-3'>
          <div className='space-y-1.5'>
            <Label className='text-muted-foreground text-xs'>제어기</Label>
            <Select
              value={controller}
              onValueChange={onControllerChange}
              disabled={running}
            >
              <SelectTrigger className='w-full' size='sm'>
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                {controllers.map((c) => (
                  <SelectItem key={c.name} value={c.name}>
                    {c.label}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          </div>

          {/* 제어기별 게인 슬라이더 */}
          {spec?.params.map((p) => (
            <div key={p.key} className='space-y-1.5'>
              <div className='flex items-center justify-between'>
                <Label className='text-muted-foreground text-xs uppercase'>
                  {p.key}
                </Label>
                <span className='text-xs tabular-nums'>
                  {(gains[p.key] ?? p.default).toFixed(0)}
                </span>
              </div>
              <Slider
                min={p.min}
                max={p.max}
                step={1}
                value={[gains[p.key] ?? p.default]}
                disabled={running}
                onValueChange={([v]) => onGainChange(p.key, v)}
              />
            </div>
          ))}

          {/* 감쇠비 / 적분 안정여유 배지 */}
          {d && (
            <div className='bg-muted space-y-1 rounded-md px-2.5 py-1.5 text-xs'>
              <div className='flex items-center justify-between'>
                <span className='text-muted-foreground tabular-nums'>
                  ζ={d.zeta.toFixed(2)} · ωn={d.wn.toFixed(1)}
                </span>
                <span className={`font-medium ${d.color}`}>{d.label}</span>
              </div>
              {d.margin != null && (
                <div className='border-border/50 flex items-center justify-between border-t pt-1'>
                  <span className='text-muted-foreground tabular-nums'>
                    적분여유 Ki/(Kp·Kd)={d.margin.toFixed(2)}
                  </span>
                  <span
                    className={`font-medium ${d.margin < 1 ? 'text-emerald-600' : 'text-red-600'}`}
                  >
                    {d.margin < 1 ? '안정' : '발산위험'}
                  </span>
                </div>
              )}
            </div>
          )}

          {/* 모델 불확실성: plant(진짜) ≠ controller(아는 모델) */}
          <div className='space-y-3 border-t pt-3'>
            <p className='text-muted-foreground text-xs font-medium'>
              모델 불확실성
            </p>
            <div className='space-y-1.5'>
              <div className='flex items-center justify-between'>
                <Label className='text-muted-foreground text-xs'>
                  페이로드 (미지 질량)
                </Label>
                <span className='text-xs tabular-nums'>
                  {payload.toFixed(1)} kg
                </span>
              </div>
              <Slider
                min={0}
                max={5}
                step={0.5}
                value={[payload]}
                disabled={running}
                onValueChange={([v]) => onPayloadChange(v)}
              />
            </div>
            <div className='space-y-1.5'>
              <div className='flex items-center justify-between'>
                <Label className='text-muted-foreground text-xs'>
                  모델 질량배율
                </Label>
                <span className='text-xs tabular-nums'>
                  ×{modelScale.toFixed(2)}
                </span>
              </div>
              <Slider
                min={0.5}
                max={1.5}
                step={0.05}
                value={[modelScale]}
                disabled={running}
                onValueChange={([v]) => onModelScaleChange(v)}
              />
            </div>
          </div>

          <Button
            className='w-full'
            size='sm'
            onClick={onRun}
            disabled={running}
          >
            {running ? '재생 중…' : 'Run ▶  home → 현재자세'}
          </Button>
          <Button
            variant='outline'
            size='sm'
            className='w-full'
            onClick={onReset}
            disabled={running}
          >
            홈 자세 (0)
          </Button>
        </div>
      </CardContent>
    </Card>
  );
}
