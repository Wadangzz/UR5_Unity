// FastAPI 백엔드 클라이언트 (axios). vite 프록시로 /api → :8500
import axios from 'axios'

const http = axios.create({ baseURL: '/api' })

export interface RobotInfo {
  name: string
  n: number
  joint_names: string[]
  home: number[]
  home_tcp: number[]
  urdf_url: string
  packages: Record<string, string>
}

export interface ControllerSpec {
  name: string
  label: string
  params: { key: string; default: number; min: number; max: number }[]
}

export interface RunRequest {
  waypoints?: number[][]            // 관절 경유점(rad)
  program_id?: string
  controller?: string              // pd | pid | computed_torque
  gains?: Record<string, number>
  gravity_comp?: boolean
  t_seg?: number
  hold?: number
}

export interface RunResponse {
  t: number[]
  theta: number[][]                // [frame][6] 관절각(rad)
  tcp: number[][]                  // [frame][3]
  error: number[]
  torque: number[][]               // [frame][6]
  waypoints_tcp: number[][]
}

// URDF 에서 추출한 관절 메타 (슬라이더 범위용, rad)
export interface JointMeta {
  name: string
  lower: number
  upper: number
}

export const getRobot = () => http.get<RobotInfo>('/robot').then((r) => r.data)
export const getControllers = () =>
  http.get<ControllerSpec[]>('/controllers').then((r) => r.data)
export const runSimulation = (req: RunRequest) =>
  http.post<RunResponse>('/run', req).then((r) => r.data)
