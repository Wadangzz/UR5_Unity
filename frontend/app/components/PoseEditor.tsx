import { useEffect, useRef, useState } from 'react';
import { Euler, MathUtils, Quaternion, Vector3 } from 'three';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Input } from '@/components/ui/input';
import { fk, ik } from '@/api';

interface Props {
  robotId: string; // IK/FK 대상 로봇
  joints: number[];
  onSolved: (theta: number[]) => void;
  running: boolean;
  live?: { pose: number[]; sigma: number } | null; // 재생 중 현재 프레임 EE pose·σ_min
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
    step: 0.5,
    color: 'text-muted-foreground',
  },
  {
    k: 'ry',
    label: 'RY',
    unit: '°',
    step: 0.5,
    color: 'text-muted-foreground',
  },
  {
    k: 'rz',
    label: 'RZ',
    unit: '°',
    step: 0.5,
    color: 'text-muted-foreground',
  },
];

// RX/RY/RZ 편집은 절대 오일러 재합성 대신 base(월드) 축 증분 회전으로 합성한다
// → 각 축이 독립적으로 동작(자유도 상실 없음 = 짐벌락 없음). 위치 X/Y/Z 와 같은 base 프레임.
const AXES: Record<'rx' | 'ry' | 'rz', Vector3> = {
  rx: new Vector3(1, 0, 0),
  ry: new Vector3(0, 1, 0),
  rz: new Vector3(0, 0, 1),
};

const fromPose7 = (pose: number[]): Pose6 => {
  const e = new Euler().setFromQuaternion(
    new Quaternion(pose[3], pose[4], pose[5], pose[6]),
    'XYZ',
  );
  return {
    x: pose[0],
    y: pose[1],
    z: pose[2],
    rx: MathUtils.radToDeg(e.x),
    ry: MathUtils.radToDeg(e.y),
    rz: MathUtils.radToDeg(e.z),
  };
};

/**
 * 직교 6-DOF(X,Y,Z + RX,RY,RZ) ↔ 관절각 양방향 연동.
 * - 정방향: joints 변경 → 디바운스 FK → 필드 표시 갱신 (jog·Run 결과 반영).
 * - 역방향: 필드 편집 → 라이브 IK → 관절각 반영 (onChange 에서만 트리거).
 * IK 를 effect 가 아니라 편집에서만 트리거 → running 변화로 stale 타깃이
 * 재적용되는 버그 없음. FK 동기화는 재생 중·IK 중·방금 편집(400ms) 중이면 건너뜀.
 */
export default function PoseEditor({
  robotId,
  joints,
  onSolved,
  running,
  live,
}: Props) {
  const [pose, setPose] = useState<Pose6 | null>(null);
  const [status, setStatus] = useState('');
  const [sigma, setSigma] = useState<number | null>(null); // σ_min

  const latest = useRef<number[] | null>(null); // 마지막 IK 목표 pose7
  const quatRef = useRef(new Quaternion()); // 현재 자세(권위 쿼터니언) — 편집은 여기에 증분
  const inFlight = useRef(false);
  const editedAt = useRef(0); // 마지막 사용자 편집 시각(ms)
  const jointsRef = useRef(joints);
  jointsRef.current = joints;
  const onSolvedRef = useRef(onSolved);
  onSolvedRef.current = onSolved;

  const pump = useRef<() => void>(() => {});
  pump.current = async () => {
    if (inFlight.current) return;
    const target = latest.current;
    if (!target) return;
    latest.current = null;
    inFlight.current = true;
    try {
      const r = await ik(target, jointsRef.current, robotId);
      if (r.converged) onSolvedRef.current(r.theta);
      setStatus(r.converged ? '도달 ✓' : '도달 불가 ✗');
    } catch {
      setStatus('오류');
    } finally {
      inFlight.current = false;
      if (latest.current) pump.current(); // trailing: 진행 중 들어온 최신 목표
    }
  };

  // 정방향 FK 동기화 (joints → 필드). 라이브: in-flight 가드 + trailing.
  const fkInFlight = useRef(false);
  const fkDirty = useRef(false);
  const fkPump = useRef<() => void>(() => {});
  fkPump.current = async () => {
    if (fkInFlight.current) return;
    fkInFlight.current = true;
    fkDirty.current = false;
    try {
      const r = await fk(jointsRef.current, robotId);
      setSigma(r.sigma_min); // 현재 자세의 특이점 근접도 (항상 갱신)
      // 사용자가 편집 중이거나 IK 진행 중이면 표시 갱신 보류 (충돌 방지)
      if (!inFlight.current && Date.now() - editedAt.current > 300) {
        quatRef.current = new Quaternion(
          r.pose[3],
          r.pose[4],
          r.pose[5],
          r.pose[6],
        );
        setPose(fromPose7(r.pose));
      }
    } catch {
      /* ignore */
    } finally {
      fkInFlight.current = false;
      if (fkDirty.current) fkPump.current(); // trailing: 그 사이 joints 변경
    }
  };
  useEffect(() => {
    if (running || joints.length === 0) return;
    fkDirty.current = true;
    fkPump.current();
  }, [joints, running]);

  // 역방향: 필드 편집 → IK (명령형, effect 아님)
  // 자세(RX/RY/RZ)는 절대 오일러를 재합성하지 않고 현재 쿼터니언에 base 축 증분 회전을
  // 곱한다 → ry≈±90° 에서도 각 축이 독립적으로 동작(짐벌락 없음). 위치는 쿼터니언 유지.
  const setField = (k: keyof Pose6, v: number) => {
    if (!pose) return;
    editedAt.current = Date.now();
    if (k === 'rx' || k === 'ry' || k === 'rz') {
      const d = MathUtils.degToRad(v - pose[k]); // 화면 readout 대비 증분각
      const qd = new Quaternion().setFromAxisAngle(AXES[k], d);
      quatRef.current = qd.multiply(quatRef.current).normalize(); // base 프레임(premultiply)
    }
    const x = k === 'x' ? v : pose.x;
    const y = k === 'y' ? v : pose.y;
    const z = k === 'z' ? v : pose.z;
    const q = quatRef.current;
    // 표시 오일러는 결과 쿼터니언에서 재유도 (특이점 근처 readout 은 튈 수 있으나 명령은 정확)
    const e = new Euler().setFromQuaternion(q, 'XYZ');
    setPose({
      x,
      y,
      z,
      rx: MathUtils.radToDeg(e.x),
      ry: MathUtils.radToDeg(e.y),
      rz: MathUtils.radToDeg(e.z),
    });
    latest.current = [x, y, z, q.x, q.y, q.z, q.w];
    pump.current();
  };

  // 재생 중엔 응답 시계열의 현재 프레임 값을, 평소엔 jog FK 결과를 표시
  const dispPose = running && live ? fromPose7(live.pose) : pose;
  const dispSigma = running && live ? live.sigma : sigma;

  if (!dispPose) return null;

  return (
    <Card className='bg-card/95 supports-[backdrop-filter]:bg-card/80 w-60 backdrop-blur'>
      <CardHeader className='pb-3'>
        <CardTitle className='text-sm'>직교 좌표 (6 DOF)</CardTitle>
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
              value={Number(dispPose[f.k].toFixed(3))}
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
        {dispSigma !== null && (
          <div className='bg-muted flex items-center justify-between rounded-md px-2.5 py-1.5 text-xs'>
            <span className='text-muted-foreground tabular-nums'>
              Manipulability σ_min={dispSigma.toFixed(3)}
            </span>
            <span
              className={`font-medium ${dispSigma < 0.03 ? 'text-red-600' : 'text-emerald-600'}`}
            >
              {dispSigma < 0.03 ? '⚠ 특이점 근접' : '정상'}
            </span>
          </div>
        )}
      </CardContent>
    </Card>
  );
}
