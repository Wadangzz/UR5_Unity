// FastAPI 백엔드 클라이언트 (axios). vite 프록시로 /api → :8500
import axios from 'axios';

const http = axios.create({ baseURL: '/api' });

export interface RobotInfo {
  name: string;
  n: number;
  joint_names: string[];
  home: number[];
  home_tcp: number[];
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
];

export interface RunRequest {
  waypoints?: number[][]; // 관절 경유점(rad)
  program_id?: string;
  controller?: string; // pd | pid | computed_torque
  gains?: Record<string, number>;
  gravity_comp?: boolean;
  t_seg?: number;
  hold?: number;
  payload?: number; // 끝단 미지 질량(kg), plant 에만
  model_scale?: number; // 컨트롤러 질량 배율(1.0=정확)
  disturbance?: number[]; // 끝단 외력(base 프레임, N), 모션 후 인가
}

export interface RunResponse {
  t: number[];
  theta: number[][]; // [frame][6] 관절각(rad)
  tcp: number[][]; // [frame][3]
  error: number[];
  torque: number[][]; // [frame][6]
  waypoints_tcp: number[][];
  settle_time: number | null; // 평형 도달 시각(s), 미도달 null
  steady_state_error: number | null; // 정상상태 오차(rad)
  diverged: boolean; // 발산 여부
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
