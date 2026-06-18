// FastAPI 백엔드 클라이언트 (axios). vite 프록시로 /api → :8500
import axios from 'axios';

const http = axios.create({ baseURL: '/api' });

export interface RobotInfo {
  name: string;
  n: number;
  joint_names: string[];
  home: number[];
  home_tcp: number[];
  ready: number[]; // 비특이 운용 출발 자세
  urdf_url: string;
  packages: Record<string, string>;
}

export interface ControllerSpec {
  name: string;
  label: string;
  params: {
    key: string;
    default: number;
    min: number;
    max: number;
    label?: string;
  }[];
}

// 제어기 스펙(정적 UI 메타데이터). 백엔드는 controller 문자열+gains 만 받으면 되므로
// 굳이 API 로 받지 않고 프론트가 보유. 게인 키(kp/ki/kd)는 백엔드 torque 와의 계약.
export const CONTROLLERS: ControllerSpec[] = [
  {
    name: 'pd',
    label: 'PD + 중력보상',
    params: [
      { key: 'kp', default: 100, min: 0, max: 400 },
      { key: 'kd', default: 20, min: 0, max: 100 },
    ],
  },
  {
    name: 'pid',
    label: 'PID',
    params: [
      { key: 'kp', default: 120, min: 0, max: 400 },
      { key: 'ki', default: 60, min: 0, max: 300 },
      { key: 'kd', default: 35, min: 0, max: 100 },
    ],
  },
  {
    name: 'computed_torque',
    label: 'Computed Torque',
    params: [
      { key: 'kp', default: 100, min: 0, max: 400 },
      { key: 'ki', default: 0, min: 0, max: 200 },
      { key: 'kd', default: 20, min: 0, max: 100 },
    ],
  },
  {
    name: 'impedance',
    label: '임피던스',
    params: [
      { key: 'kp', default: 600, min: 0, max: 3000, label: 'K (강성, N/m)' },
      { key: 'kd', default: 60, min: 0, max: 400, label: 'D (감쇠, N·s/m)' },
    ],
  },
  // 속도제어(MR §11.3): 1차계 — θ̇ 가 곧 출력. 게인 kp/ki 만(kd 없음), 스케일 작음.
  {
    name: 'joint_velocity',
    label: '관절 속도제어',
    params: [
      { key: 'kp', default: 3, min: 0, max: 10 },
      { key: 'ki', default: 0, min: 0, max: 5 },
    ],
  },
  {
    name: 'resolved_rate',
    label: 'Resolved-Rate (직교속도)',
    params: [
      { key: 'kp', default: 3, min: 0, max: 10 },
      { key: 'ki', default: 0, min: 0, max: 5 },
    ],
  },
  // 어드미턴스(MR §11.7.2): 외력→움직임. 가상 M-B-K, 외란을 f_ext 로 사용.
  {
    name: 'admittance',
    label: '어드미턴스',
    params: [
      { key: 'm', default: 2, min: 1, max: 10, label: 'M (가상질량, kg)' },
      { key: 'b', default: 40, min: 0, max: 200, label: 'B (감쇠, N·s/m)' },
      { key: 'k', default: 600, min: 0, max: 3000, label: 'K (강성, N/m)' },
    ],
  },
];

// 극배치(2차계) 자동튜닝이 의미있는 제어기. 속도제어(1차계)·임피던스(직교강성)는 제외.
export const POLE_PLACE = ['pd', 'pid', 'computed_torque'];

export interface RunRequest {
  waypoints?: number[][]; // 관절 경유점(rad)
  program_id?: string;
  start?: number[]; // 프로그램 실행 출발 관절각(현재 자세)
  controller?: string; // pd | pid | computed_torque
  gains?: Record<string, number>;
  traj_mode?: string; // joint(관절 quintic) | task(직교 직선)
  gravity_comp?: boolean;
  t_seg?: number;
  hold?: number;
  payload?: number; // 끝단 미지 질량(kg), plant 에만
  model_scale?: number; // 컨트롤러 질량 배율(1.0=정확)
  disturbance?: number[]; // 끝단 외력(base 프레임, N), 모션 후 인가
  friction?: number; // 관절 쿨롱 마찰(N·m, plant) — 토크 제어만
  tau_max?: number; // 액추에이터 토크 한계(N·m, 0=무제한) — 토크 제어만
}

export interface RunResponse {
  t: number[];
  theta: number[][]; // [frame][6] 관절각(rad)
  tcp: number[][]; // [frame][3]
  error: number[];
  torque: number[][]; // [frame][6] 토크제어 출력(N·m). 속도제어 시 빈 배열
  qdot: number[][]; // [frame][6] 속도제어 출력 commanded 관절속도(rad/s)
  ee_pose: number[][]; // [frame][7] EE pose [x,y,z,qx,qy,qz,qw] (재생 중 직교좌표)
  sigma_min: number[]; // [frame] 최소특이값 (재생 중 manipulability)
  waypoints_tcp: number[][];
  settle_time: number | null; // 평형 도달 시각(s), 미도달 null
  steady_state_error: number | null; // 정상상태 오차(rad)
  diverged: boolean; // 발산 여부
  traj_error: string | null; // task궤적 실패 사유(특이점/작업영역밖), 없으면 null
}

// URDF 에서 추출한 관절 메타 (슬라이더 범위용, rad)
export interface JointMeta {
  name: string;
  lower: number;
  upper: number;
}

export interface IKResponse {
  theta: number[];
  converged: boolean;
  tcp: number[];
}

export interface FKResponse {
  pose: number[]; // [x,y,z, qx,qy,qz,qw]
  tcp: number[];
  w: number; // 조작성 √det(JJᵀ)
  sigma_min: number; // 최소 특이값 (→0 특이점)
}

export interface Pose {
  id: number;
  program_id: string;
  x: number;
  y: number;
  z: number;
  qx: number;
  qy: number;
  qz: number;
  qw: number;
  theta: number[];
}

export const getRobot = () => http.get<RobotInfo>('/robot').then((r) => r.data);
export const runSimulation = (req: RunRequest) =>
  http.post<RunResponse>('/run', req).then((r) => r.data);

export const fk = (theta: number[]) =>
  http.post<FKResponse>('/fk', { theta }).then((r) => r.data);
export const ik = (pose: number[], seed?: number[]) =>
  http.post<IKResponse>('/ik', { pose, seed }).then((r) => r.data);

export const getProgram = (id: string) =>
  http.get<Pose[]>(`/programs/${id}`).then((r) => r.data);
export const savePose = (id: string, pose: number[], theta: number[]) => {
  const [x, y, z, qx, qy, qz, qw] = pose;
  return http
    .post<Pose>(`/programs/${id}/poses`, { x, y, z, qx, qy, qz, qw, theta })
    .then((r) => r.data);
};
export const resetProgram = (id: string) =>
  http.delete(`/programs/${id}`).then((r) => r.data);
export const deletePose = (id: string, poseId: number) =>
  http.delete(`/programs/${id}/poses/${poseId}`).then((r) => r.data);
