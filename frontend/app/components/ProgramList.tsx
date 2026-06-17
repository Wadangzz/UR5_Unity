import { useEffect, useState } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { fk, savePose, getProgram, resetProgram, type Pose } from '@/api'

const PID = 'default'

interface Props {
  joints: number[]
  onRunProgram: (programId: string) => void
  running: boolean
}

/** 포즈 티치(현재자세 저장) → 목록 → 프로그램 실행(home→순회) → 초기화. */
export default function ProgramList({ joints, onRunProgram, running }: Props) {
  const [poses, setPoses] = useState<Pose[]>([])
  const [busy, setBusy] = useState(false)

  const refresh = () => getProgram(PID).then(setPoses).catch(() => {})
  useEffect(() => {
    refresh()
  }, [])

  const save = async () => {
    setBusy(true)
    try {
      const r = await fk(joints)
      await savePose(PID, r.pose)
      await refresh()
    } finally {
      setBusy(false)
    }
  }
  const clear = async () => {
    await resetProgram(PID)
    await refresh()
  }

  return (
    <Card className="w-60 bg-card/95 backdrop-blur supports-[backdrop-filter]:bg-card/80">
      <CardHeader className="pb-3">
        <CardTitle className="text-sm">프로그램 (티치)</CardTitle>
      </CardHeader>
      <CardContent className="space-y-3">
        <ol className="max-h-32 space-y-1 overflow-y-auto text-xs">
          {poses.length === 0 && <li className="text-muted-foreground">저장된 포즈 없음</li>}
          {poses.map((p, i) => (
            <li key={p.id} className="flex justify-between tabular-nums">
              <span className="text-muted-foreground">P{i + 1}</span>
              <span>
                ({p.x.toFixed(2)}, {p.y.toFixed(2)}, {p.z.toFixed(2)})
              </span>
            </li>
          ))}
        </ol>
        <Button
          variant="outline"
          size="sm"
          className="w-full"
          onClick={save}
          disabled={running || busy}
        >
          + 현재 자세 저장
        </Button>
        <Button
          size="sm"
          className="w-full"
          onClick={() => onRunProgram(PID)}
          disabled={running || poses.length === 0}
        >
          프로그램 실행 ▶ (home→순회)
        </Button>
        <Button
          variant="ghost"
          size="sm"
          className="w-full"
          onClick={clear}
          disabled={running || poses.length === 0}
        >
          초기화
        </Button>
      </CardContent>
    </Card>
  )
}
