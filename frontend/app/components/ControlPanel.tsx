import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Slider } from '@/components/ui/slider'
import { Label } from '@/components/ui/label'
import { Button } from '@/components/ui/button'
import type { JointMeta } from '@/api'

interface Props {
  meta: JointMeta[]
  joints: number[]
  onChange: (index: number, value: number) => void
  onReset: () => void
}

const toDeg = (rad: number) => (rad * 180) / Math.PI

/** 관절 슬라이더 패널 — 드래그 시 브라우저에서 즉시 FK (백엔드 왕복 없음). */
export default function ControlPanel({ meta, joints, onChange, onReset }: Props) {
  if (meta.length === 0) return null
  return (
    <Card className="w-72 bg-card/95 backdrop-blur supports-[backdrop-filter]:bg-card/80">
      <CardHeader className="pb-3">
        <CardTitle className="text-sm">관절 제어</CardTitle>
      </CardHeader>
      <CardContent className="space-y-4">
        {meta.map((m, i) => (
          <div key={m.name} className="space-y-1.5">
            <div className="flex items-center justify-between">
              <Label className="text-xs text-muted-foreground">{m.name}</Label>
              <span className="text-xs tabular-nums">{toDeg(joints[i] ?? 0).toFixed(0)}°</span>
            </div>
            <Slider
              min={m.lower}
              max={m.upper}
              step={0.01}
              value={[joints[i] ?? 0]}
              onValueChange={([v]) => onChange(i, v)}
            />
          </div>
        ))}
        <Button variant="outline" size="sm" className="w-full" onClick={onReset}>
          홈 자세 (0)
        </Button>
      </CardContent>
    </Card>
  )
}
