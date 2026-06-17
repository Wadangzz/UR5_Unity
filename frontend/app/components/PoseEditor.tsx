import { useEffect, useState } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Input } from '@/components/ui/input'
import { Button } from '@/components/ui/button'
import { fk, ik } from '@/api'

interface Props {
  joints: number[]
  onSolved: (theta: number[]) => void
  running: boolean
}

// 축 색은 씬의 axesHelper 와 동일 (X 빨강·Y 초록·Z 파랑)
const AXES = [
  { key: 'X', i: 0, color: 'text-red-600' },
  { key: 'Y', i: 1, color: 'text-emerald-600' },
  { key: 'Z', i: 2, color: 'text-blue-600' },
]

/** 직교 목표(x,y,z) → 현재 방향 유지하여 IK → 관절각 반영. */
export default function PoseEditor({ joints, onSolved, running }: Props) {
  const [xyz, setXyz] = useState<number[] | null>(null)
  const [status, setStatus] = useState('')
  const [busy, setBusy] = useState(false)

  // 최초 1회 현재 TCP 위치로 입력칸 시드
  useEffect(() => {
    if (joints.length && !xyz) fk(joints).then((r) => setXyz(r.pose.slice(0, 3)))
  }, [joints, xyz])

  if (!xyz) return null

  const setAxis = (i: number, v: number) =>
    setXyz((prev) => (prev ? prev.map((p, k) => (k === i ? v : p)) : prev))

  const move = async () => {
    setBusy(true)
    setStatus('')
    try {
      const cur = await fk(joints) // 현재 EE 방향 유지
      const pose = [...xyz, cur.pose[3], cur.pose[4], cur.pose[5], cur.pose[6]]
      const r = await ik(pose, joints)
      if (r.converged) {
        onSolved(r.theta)
        setStatus('도달 ✓')
      } else {
        setStatus('도달 불가 ✗')
      }
    } catch {
      setStatus('오류')
    } finally {
      setBusy(false)
    }
  }

  return (
    <Card className="w-60 bg-card/95 backdrop-blur supports-[backdrop-filter]:bg-card/80">
      <CardHeader className="pb-3">
        <CardTitle className="text-sm">직교 목표 (IK)</CardTitle>
      </CardHeader>
      <CardContent className="space-y-2.5">
        {AXES.map((a) => (
          <div key={a.key} className="flex items-center gap-2">
            <span className={`w-3 text-xs font-bold ${a.color}`}>{a.key}</span>
            <Input
              type="number"
              step={0.02}
              value={xyz[a.i]}
              disabled={running || busy}
              onChange={(e) => setAxis(a.i, parseFloat(e.target.value) || 0)}
              className="h-8"
            />
            <span className="text-xs text-muted-foreground">m</span>
          </div>
        ))}
        <Button className="w-full" size="sm" onClick={move} disabled={running || busy}>
          {busy ? '풀이 중…' : '이동 (IK)'}
        </Button>
        {status && <p className="text-center text-xs text-muted-foreground">{status}</p>}
      </CardContent>
    </Card>
  )
}
