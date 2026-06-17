// FastAPI 백엔드 fetch 래퍼. (vite 프록시로 /api → :8500)

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
  params: { key: string; default: number; min: number; max: number }[]
}

async function json<T>(r: Response): Promise<T> {
  if (!r.ok) throw new Error(`${r.status} ${r.url}`)
  return r.json() as Promise<T>
}

export const getRobot = () => fetch('/api/robot').then(json<RobotInfo>)
export const getControllers = () => fetch('/api/controllers').then(json<ControllerSpec[]>)
