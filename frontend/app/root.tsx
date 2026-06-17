import { useCallback, useEffect, useRef, useState } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Grid } from "@react-three/drei";
import RobotView from "@/components/RobotView";
import ControlPanel from "@/components/ControlPanel";
import PoseEditor from "@/components/PoseEditor";
import ProgramList from "@/components/ProgramList";
import Plots from "@/components/Plots";
import {
  getControllers,
  runSimulation,
  type ControllerSpec,
  type JointMeta,
  type RunRequest,
  type RunResponse,
} from "@/api";

export default function App() {
  const [meta, setMeta] = useState<JointMeta[]>([]);
  const [joints, setJoints] = useState<number[]>([]);
  const [controllers, setControllers] = useState<ControllerSpec[]>([]);
  const [controller, setController] = useState("computed_torque");
  const [gains, setGains] = useState<Record<string, number>>({});
  const [payload, setPayload] = useState(0);
  const [modelScale, setModelScale] = useState(1);
  const [running, setRunning] = useState(false);
  const [result, setResult] = useState<RunResponse | null>(null);
  const rafRef = useRef<number | null>(null);

  useEffect(() => {
    getControllers()
      .then(setControllers)
      .catch(() => {});
  }, []);
  // 제어기 바뀌면 해당 제어기 기본 게인으로 초기화
  useEffect(() => {
    const spec = controllers.find((c) => c.name === controller);
    if (!spec) return;
    const g: Record<string, number> = {};
    spec.params.forEach((p) => (g[p.key] = p.default));
    setGains(g);
  }, [controller, controllers]);
  const setGain = (key: string, value: number) =>
    setGains((prev) => ({ ...prev, [key]: value }));
  // 언마운트 시 재생 루프 정리
  useEffect(
    () => () => {
      if (rafRef.current) cancelAnimationFrame(rafRef.current);
    },
    [],
  );

  // URDF 로드 완료 → 관절 메타 저장 + 관절각 0 으로 초기화
  const handleLoaded = useCallback((m: JointMeta[]) => {
    setMeta(m);
    setJoints(m.map(() => 0));
  }, []);

  const setJoint = (index: number, value: number) =>
    setJoints((prev) => prev.map((p, i) => (i === index ? value : p)));
  const reset = () => setJoints(meta.map(() => 0));

  // θ(t) 를 실시간으로 재생 (requestAnimationFrame, t[] 타이밍대로)
  const play = (theta: number[][], t: number[]) => {
    const dur = t[t.length - 1] || 0;
    const start = performance.now();
    const tick = (now: number) => {
      const el = (now - start) / 1000;
      let i = 0;
      while (i < t.length - 1 && t[i + 1] <= el) i++;
      setJoints(theta[i]);
      if (el < dur) {
        rafRef.current = requestAnimationFrame(tick);
      } else {
        setJoints(theta[theta.length - 1]);
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
        payload,
        model_scale: modelScale,
        ...req,
      });
      setResult(res);
      play(res.theta, res.t);
    } catch (e) {
      console.error(e);
      setRunning(false);
    }
  };
  // 현재 슬라이더 자세를 목표로 home→목표
  const run = () => runReq({ waypoints: [meta.map(() => 0), [...joints]] });
  // 티치 프로그램 순회 (위 제어기 세팅 그대로 적용)
  const runProgram = (programId: string) => runReq({ program_id: programId });

  return (
    <div className="relative h-screen w-screen bg-background">
      <header className="pointer-events-none absolute left-0 top-0 z-10 p-4">
        <h1 className="text-lg font-semibold tracking-tight text-foreground">
          UR5 Web Simulator
        </h1>
        <p className="text-sm text-muted-foreground">
          react-three-fiber · urdf-loader · FastAPI
        </p>
      </header>

      <div className="absolute right-4 top-4 z-10">
        <ControlPanel
          meta={meta}
          joints={joints}
          onChange={setJoint}
          onReset={reset}
          controllers={controllers}
          controller={controller}
          onControllerChange={setController}
          gains={gains}
          onGainChange={setGain}
          payload={payload}
          onPayloadChange={setPayload}
          modelScale={modelScale}
          onModelScaleChange={setModelScale}
          onRun={run}
          running={running}
        />
      </div>

      <div className="absolute left-4 top-20 z-10 space-y-3">
        <PoseEditor joints={joints} onSolved={setJoints} running={running} />
        <ProgramList joints={joints} onRunProgram={runProgram} running={running} />
      </div>

      <div className="absolute bottom-4 left-1/2 z-10 -translate-x-1/2">
        <Plots result={result} />
      </div>

      <Canvas camera={{ position: [1.4, 1.1, 1.4], fov: 50 }} shadows>
        <ambientLight intensity={0.7} />
        <directionalLight position={[5, 10, 5]} intensity={1.2} castShadow />
        <directionalLight position={[-5, 5, -5]} intensity={0.4} />

        <RobotView joints={joints} onLoaded={handleLoaded} />

        {/* 로봇 베이스 프레임 좌표축 (X 빨강·Y 초록·Z 파랑/위), URDF Z-up 정렬 */}
        <axesHelper args={[0.4]} rotation={[-Math.PI / 2, 0, 0]} />

        <Grid
          args={[10, 10]}
          cellSize={0.1}
          cellThickness={0.6}
          sectionSize={0.5}
          sectionThickness={1}
          infiniteGrid
          fadeDistance={8}
          cellColor="#d4d4d4"
          sectionColor="#a3a3a3"
        />
        <OrbitControls target={[0, 0.4, 0]} enableDamping />
      </Canvas>
    </div>
  );
}
