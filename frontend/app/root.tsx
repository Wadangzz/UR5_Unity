import { useCallback, useEffect, useRef, useState } from 'react';
import { Canvas } from '@react-three/fiber';
import {
  OrbitControls,
  Grid,
  GizmoHelper,
  GizmoViewport,
} from '@react-three/drei';
import RobotView from '@/components/RobotView';
import ControlPanel from '@/components/ControlPanel';
import PoseEditor from '@/components/PoseEditor';
import ProgramList from '@/components/ProgramList';
import Plots from '@/components/Plots';
import {
  CONTROLLERS,
  POLE_PLACE,
  runSimulation,
  type JointMeta,
  type RunRequest,
  type RunResponse,
} from '@/api';

export default function App() {
  const [meta, setMeta] = useState<JointMeta[]>([]);
  const [joints, setJoints] = useState<number[]>([]);
  const [ready, setReady] = useState<number[]>([]); // 비특이 운용 출발 자세
  const controllers = CONTROLLERS;
  const [controller, setController] = useState('computed_torque');
  const [trajMode, setTrajMode] = useState('joint'); // joint | task(직교 직선)
  const [gains, setGains] = useState<Record<string, number>>({});
  const [autoTune, setAutoTune] = useState(false);
  const [targetTs, setTargetTs] = useState(0.6); // 목표 정착시간(s)
  const [payload, setPayload] = useState(0);
  const [modelScale, setModelScale] = useState(1);
  const [push, setPush] = useState<number[]>([0, 0, 0]); // 외란 외력(N, base XYZ)
  const setPushAxis = (index: number, value: number) =>
    setPush((prev) => prev.map((p, i) => (i === index ? value : p)));
  const [running, setRunning] = useState(false);
  const [result, setResult] = useState<RunResponse | null>(null);
  // 재생 중 현재 프레임의 EE pose·σ_min (직교좌표/manipulability 라이브 표시용)
  const [live, setLive] = useState<{ pose: number[]; sigma: number } | null>(
    null,
  );
  const rafRef = useRef<number | null>(null);

  // 게인 결정: 자동(극배치)이면 목표 정착시간에서 계산, 수동이면 제어기 기본값.
  // 극배치(ζ=1 임계, 2% 정착 ts≈4/(ζ·ωn)): ωn=4/ts → Kp=ωn², Kd=2ωn.
  // Ki(있으면)는 안정한계 Ki<Kp·Kd 내 안전값(0.3·Kp·Kd).
  useEffect(() => {
    const spec = controllers.find((c) => c.name === controller);
    if (!spec) return;
    const g: Record<string, number> = {};
    // 극배치 자동튜닝은 관절 2차계용 → 속도제어(1차계)·임피던스(직교강성)는 기본값/수동
    if (autoTune && POLE_PLACE.includes(controller)) {
      const wn = 4 / targetTs;
      const kp = wn * wn;
      const kd = 2 * wn;
      spec.params.forEach((p) => {
        const v =
          p.key === 'kp'
            ? kp
            : p.key === 'kd'
              ? kd
              : p.key === 'ki'
                ? 0.3 * kp * kd
                : p.default;
        g[p.key] = Math.min(p.max, Math.max(p.min, v));
      });
    } else {
      spec.params.forEach((p) => (g[p.key] = p.default));
    }
    setGains(g);
  }, [controller, controllers, autoTune, targetTs]);
  const setGain = (key: string, value: number) =>
    setGains((prev) => ({ ...prev, [key]: value }));
  // 언마운트 시 재생 루프 정리
  useEffect(
    () => () => {
      if (rafRef.current) cancelAnimationFrame(rafRef.current);
    },
    [],
  );

  // URDF 로드 완료 → 관절 메타 저장 + 비특이 ready 자세로 초기화
  // (kinematic zero=전관절0 은 특이점이라 시작 자세로 부적절)
  const handleLoaded = useCallback((m: JointMeta[], r: number[]) => {
    setMeta(m);
    setReady(r);
    setJoints(r);
  }, []);

  const setJoint = (index: number, value: number) =>
    setJoints((prev) => prev.map((p, i) => (i === index ? value : p)));
  const reset = () => setJoints(meta.map(() => 0));

  // θ(t) 를 실시간으로 재생 (requestAnimationFrame, t[] 타이밍대로)
  // 프레임마다 EE pose·σ_min 도 같이 갱신 → 직교좌표/manipulability 라이브 표시
  const play = (res: RunResponse) => {
    const { theta, t, ee_pose, sigma_min } = res;
    if (theta.length === 0) {
      setRunning(false); // 궤적 생성 실패 등 재생할 프레임 없음
      return;
    }
    const showFrame = (i: number) => {
      setJoints(theta[i]);
      if (ee_pose[i]) setLive({ pose: ee_pose[i], sigma: sigma_min[i] });
    };
    const dur = t[t.length - 1] || 0;
    const start = performance.now();
    const tick = (now: number) => {
      const el = (now - start) / 1000;
      let i = 0;
      while (i < t.length - 1 && t[i + 1] <= el) i++;
      showFrame(i);
      if (el < dur) {
        rafRef.current = requestAnimationFrame(tick);
      } else {
        showFrame(theta.length - 1);
        setRunning(false);
      }
    };
    rafRef.current = requestAnimationFrame(tick);
  };

  // 공통 실행: 현재 제어기/게인/모델불확실성 세팅을 실어 시뮬 → 재생
  const runReq = async (req: Partial<RunRequest>) => {
    if (running || meta.length === 0) return;
    setRunning(true);
    try {
      const res = await runSimulation({
        controller,
        gains,
        traj_mode: trajMode,
        payload,
        model_scale: modelScale,
        disturbance: push,
        ...req,
      });
      setResult(res);
      play(res);
    } catch (e) {
      console.error(e);
      setRunning(false);
    }
  };
  // 현재 슬라이더 자세를 목표로 ready(비특이)→목표
  const run = () => runReq({ waypoints: [ready, [...joints]] });
  // 티치 프로그램 순회 (현재 자세에서 출발 — 위 제어기 세팅 그대로 적용)
  const runProgram = (programId: string) =>
    runReq({ program_id: programId, start: [...joints] });

  return (
    <div className='bg-background relative h-screen w-screen'>
      <header className='pointer-events-none absolute top-0 left-0 z-10 p-4'>
        <h1 className='text-foreground text-lg font-semibold tracking-tight'>
          UR5 Web Simulator
        </h1>
        <p className='text-muted-foreground text-sm'>
          react-three-fiber · urdf-loader · FastAPI
        </p>
      </header>

      <div className='absolute top-4 right-4 z-10'>
        <ControlPanel
          meta={meta}
          joints={joints}
          onChange={setJoint}
          onReset={reset}
          onReady={() => setJoints(ready)}
          controllers={controllers}
          controller={controller}
          onControllerChange={setController}
          trajMode={trajMode}
          onTrajModeChange={setTrajMode}
          gains={gains}
          onGainChange={setGain}
          autoTune={autoTune}
          onAutoTuneChange={setAutoTune}
          targetTs={targetTs}
          onTargetTsChange={setTargetTs}
          payload={payload}
          onPayloadChange={setPayload}
          modelScale={modelScale}
          onModelScaleChange={setModelScale}
          push={push}
          onPushAxisChange={setPushAxis}
          onRun={run}
          running={running}
        />
      </div>

      <div className='absolute top-20 left-4 z-10 space-y-3'>
        <PoseEditor
          joints={joints}
          onSolved={setJoints}
          running={running}
          live={live}
        />
        <ProgramList
          joints={joints}
          onRunProgram={runProgram}
          running={running}
        />
      </div>

      <div className='absolute bottom-4 left-1/2 z-10 -translate-x-1/2'>
        <Plots result={result} />
      </div>

      <Canvas
        flat
        camera={{ position: [1.3, -1.3, 0.9], up: [0, 0, 1], fov: 50 }}
        shadows
      >
        {/* flat = 톤매핑 끔(어두운 재질이 검게 뭉개지는 것 방지).
            어두운 관절부가 보이도록 ambient↑ + hemisphere + 다방향 필라이트 */}
        <ambientLight intensity={1.0} />
        <hemisphereLight
          intensity={0.8}
          color='#ffffff'
          groundColor='#777777'
        />
        <directionalLight position={[4, -4, 8]} intensity={1.6} castShadow />
        <directionalLight position={[-5, 4, 4]} intensity={0.9} />
        <directionalLight position={[0, 6, 2]} intensity={0.5} />
        <directionalLight position={[0, -5, -2]} intensity={0.4} />

        <RobotView joints={joints} onLoaded={handleLoaded} />

        {/* 로봇 베이스 프레임 좌표축 (Z-up: X 빨강·Y 초록·Z 파랑) */}
        <axesHelper args={[0.4]} />

        {/* 바닥 그리드 (XY 평면, Z=0) */}
        <Grid
          args={[10, 10]}
          rotation={[Math.PI / 2, 0, 0]}
          cellSize={0.1}
          cellThickness={0.6}
          sectionSize={0.5}
          sectionThickness={1}
          infiniteGrid
          fadeDistance={8}
          cellColor='#d4d4d4'
          sectionColor='#a3a3a3'
        />

        {/* 좌측 하단 XYZ 축 기즈모 (라벨) */}
        <GizmoHelper alignment='bottom-left' margin={[80, 80]}>
          <GizmoViewport
            axisColors={['#ef4444', '#10b981', '#3b82f6']}
            labelColor='black'
          />
        </GizmoHelper>

        <OrbitControls target={[0, 0, 0.4]} enableDamping makeDefault />
      </Canvas>
    </div>
  );
}
