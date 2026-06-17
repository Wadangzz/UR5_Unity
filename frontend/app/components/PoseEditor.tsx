import { useEffect, useRef, useState } from 'react';
import { Euler, MathUtils, Quaternion } from 'three';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Input } from '@/components/ui/input';
import { fk, ik } from '@/api';

interface Props {
  joints: number[];
  onSolved: (theta: number[]) => void;
  running: boolean;
}

interface Pose6 {
  x: number;
  y: number;
  z: number;
  rx: number; // deg
  ry: number; // deg
  rz: number; // deg
}

// X/Y/Z 색은 씬 axesHelper 와 동일. RX/RY/RZ 는 자세(Euler).
const FIELDS: {
  k: keyof Pose6;
  label: string;
  unit: string;
  step: number;
  color: string;
}[] = [
  { k: 'x', label: 'X', unit: 'm', step: 0.002, color: 'text-red-600' },
  { k: 'y', label: 'Y', unit: 'm', step: 0.002, color: 'text-emerald-600' },
  { k: 'z', label: 'Z', unit: 'm', step: 0.002, color: 'text-blue-600' },
  {
    k: 'rx',
    label: 'RX',
    unit: '°',
    step: 0.1,
    color: 'text-muted-foreground',
  },
  {
    k: 'ry',
    label: 'RY',
    unit: '°',
    step: 0.1,
    color: 'text-muted-foreground',
  },
  {
    k: 'rz',
    label: 'RZ',
    unit: '°',
    step: 0.1,
    color: 'text-muted-foreground',
  },
];

const toPose7 = (p: Pose6): number[] => {
  const q = new Quaternion().setFromEuler(
    new Euler(
      MathUtils.degToRad(p.rx),
      MathUtils.degToRad(p.ry),
      MathUtils.degToRad(p.rz),
      'XYZ',
    ),
  );
  return [p.x, p.y, p.z, q.x, q.y, q.z, q.w];
};

/**
 * 직교 6-DOF 목표(X,Y,Z + RX,RY,RZ) → 라이브 IK → 관절각 반영.
 * 폭주·경합 방지: 동시에 IK 요청 1개만(in-flight 가드), 진행 중 변경은
 * 최신값만 남겼다가 응답 후 1번 더 발사(trailing) → 항상 최신으로 수렴.
 */
export default function PoseEditor({ joints, onSolved, running }: Props) {
  const [pose, setPose] = useState<Pose6 | null>(null);
  const [status, setStatus] = useState('');

  const latest = useRef<number[] | null>(null); // 마지막 목표 pose7
  const inFlight = useRef(false);
  const jointsRef = useRef(joints);
  jointsRef.current = joints;
  const onSolvedRef = useRef(onSolved);
  onSolvedRef.current = onSolved;

  // 최초 1회 현재 자세(위치+Euler)로 시드
  useEffect(() => {
    if (joints.length && !pose) {
      fk(joints).then((r) => {
        const e = new Euler().setFromQuaternion(
          new Quaternion(r.pose[3], r.pose[4], r.pose[5], r.pose[6]),
          'XYZ',
        );
        setPose({
          x: r.pose[0],
          y: r.pose[1],
          z: r.pose[2],
          rx: MathUtils.radToDeg(e.x),
          ry: MathUtils.radToDeg(e.y),
          rz: MathUtils.radToDeg(e.z),
        });
      });
    }
  }, [joints, pose]);

  const pumpRef = useRef<() => void>(() => {});
  pumpRef.current = async () => {
    if (inFlight.current) return;
    const target = latest.current;
    if (!target) return;
    latest.current = null;
    inFlight.current = true;
    try {
      const r = await ik(target, jointsRef.current);
      if (r.converged) onSolvedRef.current(r.theta);
      setStatus(r.converged ? '도달 ✓' : '도달 불가 ✗');
    } catch {
      setStatus('오류');
    } finally {
      inFlight.current = false;
      if (latest.current) pumpRef.current(); // 진행 중 들어온 최신 목표를 1번 더 (trailing)
    }
  };

  // pose 변경 시 라이브 IK (in-flight 가드가 폭주·경합 차단)
  useEffect(() => {
    if (!pose || running) return;
    latest.current = toPose7(pose);
    pumpRef.current();
  }, [pose, running]);

  const setField = (k: keyof Pose6, v: number) =>
    setPose((prev) => (prev ? { ...prev, [k]: v } : prev));

  if (!pose) return null;

  return (
    <Card className='bg-card/95 supports-[backdrop-filter]:bg-card/80 w-60 backdrop-blur'>
      <CardHeader className='pb-3'>
        <CardTitle className='text-sm'>직교 목표 (라이브 IK)</CardTitle>
      </CardHeader>
      <CardContent className='space-y-2'>
        {FIELDS.map((f) => (
          <div key={f.k} className='flex items-center gap-2'>
            <span className={`w-5 text-xs font-bold ${f.color}`}>
              {f.label}
            </span>
            <Input
              type='number'
              step={f.step}
              value={Number(pose[f.k].toFixed(3))}
              disabled={running}
              onChange={(e) => setField(f.k, parseFloat(e.target.value) || 0)}
              className='h-8'
            />
            <span className='text-muted-foreground w-3 text-xs'>{f.unit}</span>
          </div>
        ))}
        {status && (
          <p className='text-muted-foreground text-center text-xs'>{status}</p>
        )}
      </CardContent>
    </Card>
  );
}
