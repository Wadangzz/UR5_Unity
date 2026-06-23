import { useCallback, useEffect, useRef, useState } from 'react';
import { Link } from 'react-router-dom';
import { Canvas, type ThreeEvent } from '@react-three/fiber';
import {
  OrbitControls,
  Grid,
  GizmoHelper,
  GizmoViewport,
  Line,
} from '@react-three/drei';
import {
  Box3,
  DoubleSide,
  Mesh,
  MeshStandardMaterial,
  type Object3D,
  Quaternion,
  Vector3,
} from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import RobotView from '@/components/RobotView';
import WallPlane from '@/components/WallPlane';
import ControlPanel from '@/components/ControlPanel';
import JointPanel from '@/components/JointPanel';
import PoseEditor from '@/components/PoseEditor';
import ProgramList from '@/components/ProgramList';
import Plots from '@/components/Plots';
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select';
import {
  CONTROLLERS,
  POLE_PLACE,
  fk,
  getRobots,
  runSimulation,
  uploadMesh,
  type ForceTask,
  type JointMeta,
  type RobotSummary,
  type RunRequest,
  type RunResponse,
} from '@/api';

type Wall = {
  shape: string;
  point?: number[];
  normal?: number[];
  center?: number[];
  axis?: number[];
  radius?: number;
};

// EE pose + 표면 설정 → 표면 기하(백엔드 controller 와 동일 계산). 그리기 미리보기용.
function computeSurface(
  pose: number[],
  shape: string,
  gap: number,
  radius: number,
): Wall {
  const p = new Vector3(pose[0], pose[1], pose[2]);
  const q = new Quaternion(pose[3], pose[4], pose[5], pose[6]);
  const push = new Vector3(0, 0, 1).applyQuaternion(q).normalize();
  if (shape === 'cylinder' || shape === 'sphere') {
    const center = p.clone().addScaledVector(push, gap + radius);
    const w: Wall = { shape, center: center.toArray(), radius };
    if (shape === 'cylinder')
      w.axis = new Vector3(1, 0, 0).applyQuaternion(q).normalize().toArray();
    return w;
  }
  return {
    shape: 'plane',
    point: p.clone().addScaledVector(push, gap).toArray(),
    normal: push.clone().negate().toArray(),
  };
}

// 업로드 메시(STL/OBJ) — id·로컬 url·로컬 bbox(중심/치수). 배치는 EE 자세로 자동 계산.
type MeshInfo = {
  meshId: string;
  url: string; // object URL (로컬 렌더용)
  ext: string; // stl | obj
  center: number[]; // 로컬 bbox 중심
  size: number[]; // 로컬 bbox 치수
  n_faces: number;
  extents: number[];
};

// STL/OBJ → three Object3D (geometry 만, 재질은 호출측). bbox·렌더 공용.
function loadMeshObject(url: string, ext: string): Promise<Object3D> {
  return new Promise((resolve, reject) => {
    if (ext === 'obj') {
      new OBJLoader().load(url, resolve, undefined, reject);
    } else {
      new STLLoader().load(
        url,
        (geo) => resolve(new Mesh(geo)),
        undefined,
        reject,
      );
    }
  });
}

// EE 자세 + gap + 스케일 → 메시 배치(pos/quat). 곡면 자동배치와 같은 원리:
// bbox 중심을 도구축 앞 (gap + 두께/2) 에 둬 가까운 면이 gap 거리에 서게 한다.
// 백엔드도 같은 mesh_pos/quat 를 raw 메시에 적용하므로 렌더·시뮬이 정확히 일치.
function computeMeshPlacement(
  pose: number[],
  gap: number,
  scale: number,
  info: MeshInfo,
): { pos: number[]; quat: number[] } {
  const p = new Vector3(pose[0], pose[1], pose[2]);
  const q = new Quaternion(pose[3], pose[4], pose[5], pose[6]);
  const push = new Vector3(0, 0, 1).applyQuaternion(q).normalize();
  const halfZ = (info.size[2] / 2) * scale;
  const target = p.clone().addScaledVector(push, gap + halfZ);
  const c = new Vector3(...info.center)
    .multiplyScalar(scale)
    .applyQuaternion(q);
  return { pos: target.sub(c).toArray(), quat: [q.x, q.y, q.z, q.w] };
}

// 업로드 메시를 Z-up 씬에 반투명 렌더. 하이브리드면 클릭해 경로 그리기(world 교점).
function UploadedMesh({
  info,
  pos,
  quat,
  scale,
  onDraw,
}: {
  info: MeshInfo;
  pos: number[];
  quat: number[];
  scale: number;
  onDraw?: (p: number[]) => void;
}) {
  const [obj, setObj] = useState<Object3D | null>(null);
  useEffect(() => {
    let cancelled = false;
    loadMeshObject(info.url, info.ext)
      .then((o) => {
        if (cancelled) return;
        const mat = new MeshStandardMaterial({
          color: '#3b82f6',
          transparent: true,
          opacity: 0.3,
          side: DoubleSide,
        });
        o.traverse((c) => {
          if (c instanceof Mesh) c.material = mat;
        });
        setObj(o);
      })
      .catch(() => {});
    return () => {
      cancelled = true;
    };
  }, [info.url, info.ext]);
  if (!obj) return null;
  return (
    <primitive
      object={obj}
      position={pos}
      quaternion={quat}
      scale={scale}
      onClick={
        onDraw
          ? (e: ThreeEvent<MouseEvent>) => {
              e.stopPropagation();
              onDraw([e.point.x, e.point.y, e.point.z]);
            }
          : undefined
      }
    />
  );
}

// 사용자가 표면에 찍은 경로 점(빨강 구) + 잇는 선
function DrawnPath({ points }: { points: number[][] }) {
  if (points.length === 0) return null;
  return (
    <>
      {points.map((pt, i) => (
        <mesh key={i} position={pt as [number, number, number]}>
          <sphereGeometry args={[0.008, 16, 16]} />
          <meshStandardMaterial color='#ef4444' />
        </mesh>
      ))}
      {points.length > 1 && (
        <Line
          points={points as [number, number, number][]}
          color='#ef4444'
          lineWidth={2}
        />
      )}
    </>
  );
}

export default function App() {
  // 멀티로봇: 목록 + 선택. robotId 가 바뀌면 RobotView 를 key 로 remount → 재로드.
  const [robots, setRobots] = useState<RobotSummary[]>([]);
  const [robotId, setRobotId] = useState('ur5');
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
  const [friction, setFriction] = useState(0); // 관절 쿨롱 마찰(N·m)
  const [tauMax, setTauMax] = useState(0); // 토크 한계(N·m, 0=무제한)
  const [controlRate, setControlRate] = useState(0); // 0=연속, >0=이산 ZOH(Hz)
  const [noise, setNoise] = useState(0); // 센서 노이즈 std(rad)
  const [noiseSeed, setNoiseSeed] = useState(0); // 노이즈 realization seed
  // 힘/하이브리드: 표면·목표 (fd 목표힘, gap 접근간격, move_len 접선이동, shape 표면, radius 곡면반경)
  const [forceTask, setForceTask] = useState({
    fd: 20,
    gap: 0.05,
    move_len: 0.1,
    shape: 'plane', // plane | cylinder | sphere | mesh (컨투어)
    radius: 0.08, // 곡면 반경(m)
    mesh_scale: 1, // 업로드 메시 배치 스케일
  });
  const setForceTaskField = (
    key: 'fd' | 'gap' | 'move_len' | 'radius' | 'shape' | 'mesh_scale',
    value: number | string,
  ) => {
    setForceTask((prev) => ({ ...prev, [key]: value }));
    if (key === 'shape') setDrawnPath([]); // 표면 바뀌면 그린 경로 무효
  };
  // 업로드 메시 + 그 배치(현재 자세 기준 자동 계산). 메시 컨투어 표면.
  const [meshInfo, setMeshInfo] = useState<MeshInfo | null>(null);
  const [meshPlacement, setMeshPlacement] = useState<{
    pos: number[];
    quat: number[];
  } | null>(null);
  const onMeshUpload = async (file: File) => {
    try {
      const res = await uploadMesh(file); // 백엔드 저장 → mesh_id
      const url = URL.createObjectURL(file); // 로컬 렌더용
      const ext = file.name.split('.').pop()?.toLowerCase() ?? 'stl';
      const box = new Box3().setFromObject(await loadMeshObject(url, ext));
      const c = new Vector3();
      const s = new Vector3();
      box.getCenter(c);
      box.getSize(s);
      setMeshInfo({
        meshId: res.mesh_id,
        url,
        ext,
        center: c.toArray(),
        size: s.toArray(),
        n_faces: res.n_faces,
        extents: res.extents,
      });
      setDrawnPath([]); // 새 메시 → 이전 경로 무효
    } catch (e) {
      console.error(e);
    }
  };
  // 하이브리드 컨투어: 표면 미리보기(현재 자세 기준) + 표면에 그린 경로 점
  const [previewWall, setPreviewWall] = useState<Wall | null>(null);
  const [drawnPath, setDrawnPath] = useState<number[][]>([]);
  const addDrawPoint = (p: number[]) => setDrawnPath((prev) => [...prev, p]);
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

  // 사용 가능한 로봇 목록 (로드 없이 메타만)
  useEffect(() => {
    getRobots()
      .then(setRobots)
      .catch(() => {});
  }, []);

  // 힘/하이브리드 설정 중: 현재 자세 기준 표면 미리보기 (그리기 타깃). 디바운스.
  // 메시는 미리보기 대신 배치(pos/quat)를 계산. 재생 중엔 마지막 값을 동결(렌더 유지).
  useEffect(() => {
    const isForce = controller === 'force' || controller === 'hybrid';
    if (!isForce || joints.length === 0) {
      setPreviewWall(null);
      setMeshPlacement(null);
      return;
    }
    if (running) return; // 재생 중엔 직전 미리보기/배치 유지
    const id = setTimeout(() => {
      fk(joints, robotId)
        .then((r) => {
          if (forceTask.shape === 'mesh') {
            setPreviewWall(null);
            if (meshInfo)
              setMeshPlacement(
                computeMeshPlacement(
                  r.pose,
                  forceTask.gap,
                  forceTask.mesh_scale,
                  meshInfo,
                ),
              );
          } else {
            setMeshPlacement(null);
            setPreviewWall(
              computeSurface(
                r.pose,
                forceTask.shape,
                forceTask.gap,
                forceTask.radius,
              ),
            );
          }
        })
        .catch(() => {});
    }, 150);
    return () => clearTimeout(id);
  }, [
    joints,
    controller,
    forceTask.shape,
    forceTask.gap,
    forceTask.radius,
    forceTask.mesh_scale,
    meshInfo,
    robotId,
    running,
  ]);

  // 로봇 전환: 이전 결과/라이브 표시 비우고 robotId 변경(→ RobotView remount).
  // 관절각은 RobotView 가 새 로봇 ready 를 onLoaded 로 보고하면 거기서 초기화된다.
  const onRobotChange = (id: string) => {
    if (id === robotId || running) return;
    setRobotId(id);
    setResult(null);
    setLive(null);
  };

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
    // 실제 센서처럼 매 실행마다 다른 노이즈 realization (seed 자동 증가)
    const seed = noiseSeed + 1;
    setNoiseSeed(seed);
    try {
      const res = await runSimulation({
        robot_id: robotId,
        controller,
        gains,
        traj_mode: trajMode,
        payload,
        model_scale: modelScale,
        disturbance: push,
        friction,
        tau_max: tauMax,
        control_rate: controlRate,
        noise,
        noise_seed: seed,
        force_task: forceTask,
        ...req,
      });
      setResult(res);
      play(res);
    } catch (e) {
      console.error(e);
      setRunning(false);
    }
  };
  // 힘/하이브리드는 현재 자세에서 누름(출발=현재). 하이브리드+그린경로면 컨투어 추종.
  const isForceCtrl = controller === 'force' || controller === 'hybrid';
  // 메시 표면이면 업로드 id + 자동배치(현재 자세 기준)를 force_task 에 실어 보낸다.
  const buildForceTask = (): ForceTask => {
    if (forceTask.shape === 'mesh' && meshInfo && meshPlacement) {
      return {
        ...forceTask,
        mesh_id: meshInfo.meshId,
        mesh_pos: meshPlacement.pos,
        mesh_quat: meshPlacement.quat,
      };
    }
    return forceTask;
  };
  const run = () => {
    const ft = buildForceTask();
    if (controller === 'hybrid' && drawnPath.length > 0) {
      runReq({
        waypoints: [[...joints]],
        force_path: drawnPath,
        force_task: ft,
      });
    } else {
      runReq(
        isForceCtrl
          ? { waypoints: [[...joints]], force_task: ft }
          : { waypoints: [ready, [...joints]] },
      );
    }
  };
  // 티치 프로그램 순회 (현재 자세에서 출발 — 위 제어기 세팅 그대로 적용)
  const runProgram = (programId: string) =>
    runReq({ program_id: programId, start: [...joints] });

  return (
    <div className='bg-background relative h-screen w-screen'>
      <header className='pointer-events-none absolute top-0 left-0 z-10 p-4'>
        <h1 className='text-foreground text-lg font-semibold tracking-tight'>
          Robot Web Simulator
        </h1>
        <p className='text-muted-foreground text-sm'>
          react-three-fiber · urdf-loader · FastAPI
        </p>
      </header>

      {/* 발표자료 링크 — 우하단(상단은 패널에 가려짐) */}
      <Link
        to='/slides'
        className='bg-card/95 supports-[backdrop-filter]:bg-card/80 text-primary pointer-events-auto absolute right-4 bottom-4 z-20 inline-block rounded-lg border px-3 py-1.5 text-sm shadow-sm backdrop-blur hover:underline'
      >
        📊 발표자료 →
      </Link>

      {/* 로봇 선택 (멀티로봇) — 상단 중앙. 바꾸면 메시·기구학·동역학이 그 로봇으로 전환 */}
      <div className='absolute top-4 left-1/2 z-20 -translate-x-1/2'>
        <div className='bg-card/95 supports-[backdrop-filter]:bg-card/80 flex items-center gap-2 rounded-lg border px-3 py-1.5 shadow-sm backdrop-blur'>
          <span className='text-muted-foreground text-xs font-medium'>
            로봇
          </span>
          <Select
            value={robotId}
            onValueChange={onRobotChange}
            disabled={running}
          >
            <SelectTrigger size='sm' className='w-44 border-0 bg-transparent'>
              <SelectValue placeholder='로봇 선택' />
            </SelectTrigger>
            <SelectContent>
              {robots.map((r) => (
                <SelectItem key={r.id} value={r.id}>
                  {r.name} · {r.dof}-DOF
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>
      </div>

      {/* 하이브리드 컨투어: 표면에 마우스로 경로 그리기 안내 + 점수/지우기 */}
      {controller === 'hybrid' && !running && (
        <div className='absolute top-16 left-1/2 z-20 -translate-x-1/2'>
          <div className='bg-card/95 supports-[backdrop-filter]:bg-card/80 flex items-center gap-3 rounded-lg border px-3 py-1.5 text-xs shadow-sm backdrop-blur'>
            <span className='text-muted-foreground'>
              🖱️ 표면을 클릭해 경로를 그리세요 ·{' '}
              <span className='font-medium text-red-600'>
                {drawnPath.length}점
              </span>
            </span>
            <button
              type='button'
              onClick={() => setDrawnPath([])}
              disabled={drawnPath.length === 0}
              className='text-muted-foreground hover:text-foreground pointer-events-auto disabled:opacity-40'
            >
              지우기
            </button>
          </div>
        </div>
      )}

      <div className='absolute top-4 right-4 z-10'>
        <ControlPanel
          meta={meta}
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
          friction={friction}
          onFrictionChange={setFriction}
          tauMax={tauMax}
          onTauMaxChange={setTauMax}
          controlRate={controlRate}
          onControlRateChange={(v) => {
            setControlRate(v);
            if (v === 0) setNoise(0); // 연속은 노이즈 의미 없음 → 리셋
          }}
          noise={noise}
          onNoiseChange={setNoise}
          noiseSeed={noiseSeed}
          forceTask={forceTask}
          onForceTaskChange={setForceTaskField}
          onMeshUpload={onMeshUpload}
          meshInfo={meshInfo}
          push={push}
          onPushAxisChange={setPushAxis}
          onRun={run}
          running={running}
        />
      </div>

      <div className='absolute top-20 left-4 z-10 max-h-[calc(100vh-6rem)] space-y-3 overflow-y-auto pr-1'>
        <div className='flex items-start gap-3'>
          <PoseEditor
            robotId={robotId}
            joints={joints}
            onSolved={setJoints}
            running={running}
            live={live}
          />
          <JointPanel
            meta={meta}
            joints={joints}
            onChange={setJoint}
            running={running}
          />
        </div>
        <ProgramList
          robotId={robotId}
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

        <RobotView
          key={robotId}
          robotId={robotId}
          joints={joints}
          onLoaded={handleLoaded}
        />

        {/* 힘/하이브리드 표면 (설정 중=미리보기, 재생 중=실제). 하이브리드면 클릭해 경로 그리기 */}
        <WallPlane
          wall={running ? result?.wall : previewWall}
          onDraw={
            !running && controller === 'hybrid' ? addDrawPoint : undefined
          }
        />
        {/* 업로드 메시 표면 (shape='mesh'). 자동배치, 하이브리드면 클릭해 경로 그리기 */}
        {forceTask.shape === 'mesh' && meshInfo && meshPlacement && (
          <UploadedMesh
            info={meshInfo}
            pos={meshPlacement.pos}
            quat={meshPlacement.quat}
            scale={forceTask.mesh_scale}
            onDraw={
              !running && controller === 'hybrid' ? addDrawPoint : undefined
            }
          />
        )}
        <DrawnPath points={drawnPath} />

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
