import { useEffect, useState } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import {
  fk,
  savePose,
  getProgram,
  resetProgram,
  deletePose,
  type Pose,
} from '@/api';

interface Props {
  robotId: string; // 티치 FK 대상 + 프로그램 식별(로봇마다 독립 프로그램)
  joints: number[];
  onRunProgram: (programId: string) => void;
  running: boolean;
}

/** 포즈 티치(현재자세 저장) → 목록 → 프로그램 실행(home→순회) → 초기화. */
export default function ProgramList({
  robotId,
  joints,
  onRunProgram,
  running,
}: Props) {
  const [poses, setPoses] = useState<Pose[]>([]);
  const [busy, setBusy] = useState(false);

  // 프로그램은 로봇별로 독립 (program_id = robotId) → 로봇 바꾸면 그 로봇 프로그램만.
  const refresh = () =>
    getProgram(robotId)
      .then(setPoses)
      .catch(() => {});
  useEffect(() => {
    refresh();
  }, [robotId]);

  const save = async () => {
    setBusy(true);
    try {
      const r = await fk(joints, robotId); // Cartesian(표시·이식) + 현재 관절각(정확 재현)
      await savePose(robotId, r.pose, joints);
      await refresh();
    } finally {
      setBusy(false);
    }
  };
  const clear = async () => {
    await resetProgram(robotId);
    await refresh();
  };
  const del = async (poseId: number) => {
    await deletePose(robotId, poseId);
    await refresh();
  };

  return (
    <Card className='bg-card/95 supports-[backdrop-filter]:bg-card/80 w-60 backdrop-blur'>
      <CardHeader className='pb-3'>
        <CardTitle className='text-sm'>
          프로그램 (Teaching)
          <span className='text-muted-foreground ml-1 font-normal'>
            · {robotId}
          </span>
        </CardTitle>
      </CardHeader>
      <CardContent className='space-y-3'>
        <ol className='max-h-32 space-y-1 overflow-y-auto text-xs'>
          {poses.length === 0 && (
            <li className='text-muted-foreground'>저장된 포즈 없음</li>
          )}
          {poses.map((p, i) => (
            <li
              key={p.id}
              className='flex items-center justify-between gap-2 tabular-nums'
            >
              <span className='text-muted-foreground'>P{i + 1}</span>
              <span className='flex-1 text-right'>
                ({p.x.toFixed(2)}, {p.y.toFixed(2)}, {p.z.toFixed(2)})
              </span>
              <button
                type='button'
                onClick={() => del(p.id)}
                disabled={running}
                className='text-muted-foreground hover:text-red-600 disabled:opacity-40'
                aria-label='삭제'
              >
                ×
              </button>
            </li>
          ))}
        </ol>
        <Button
          variant='outline'
          size='sm'
          className='w-full'
          onClick={save}
          disabled={running || busy}
        >
          + 현재 자세 저장
        </Button>
        <Button
          size='sm'
          className='w-full'
          onClick={() => onRunProgram(robotId)}
          disabled={running || poses.length === 0}
        >
          프로그램 실행
        </Button>
        <Button
          variant='ghost'
          size='sm'
          className='w-full'
          onClick={clear}
          disabled={running || poses.length === 0}
        >
          초기화
        </Button>
      </CardContent>
    </Card>
  );
}
