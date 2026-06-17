import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Slider } from '@/components/ui/slider'
import { Label } from '@/components/ui/label'
import { Button } from '@/components/ui/button'
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select'
import type { ControllerSpec, JointMeta } from '@/api'

interface Props {
  meta: JointMeta[]
  joints: number[]
  onChange: (index: number, value: number) => void
  onReset: () => void
  controllers: ControllerSpec[]
  controller: string
  onControllerChange: (value: string) => void
  onRun: () => void
  running: boolean
}

const toDeg = (rad: number) => (rad * 180) / Math.PI

/** 관절 슬라이더 + 제어기 선택 + Run 패널. */
export default function ControlPanel({
  meta,
  joints,
  onChange,
  onReset,
  controllers,
  controller,
  onControllerChange,
  onRun,
  running,
}: Props) {
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
              disabled={running}
              onValueChange={([v]) => onChange(i, v)}
            />
          </div>
        ))}

        <div className="space-y-2 border-t pt-3">
          <Label className="text-xs text-muted-foreground">제어기</Label>
          <Select value={controller} onValueChange={onControllerChange} disabled={running}>
            <SelectTrigger className="w-full" size="sm">
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

          <Button className="w-full" size="sm" onClick={onRun} disabled={running}>
            {running ? '재생 중…' : 'Run ▶  home → 현재자세'}
          </Button>
          <Button
            variant="outline"
            size="sm"
            className="w-full"
            onClick={onReset}
            disabled={running}
          >
            홈 자세 (0)
          </Button>
        </div>
      </CardContent>
    </Card>
  )
}
