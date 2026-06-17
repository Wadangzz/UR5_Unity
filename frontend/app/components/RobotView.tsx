import { useEffect, useRef, useState } from 'react'
import { LoadingManager, Mesh, MeshPhongMaterial, type Object3D } from 'three'
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import URDFLoader from 'urdf-loader'
import type { URDFRobot } from 'urdf-loader'
import { getRobot, type JointMeta } from '@/api'

// urdf-loader 콜백: done(mesh, err) — 에러 시 mesh 자리는 무시되므로 null 을 캐스팅해 넘긴다.
type MeshDone = (mesh: Object3D, err?: Error) => void
const fail = (done: MeshDone, e: unknown) =>
  done(null as unknown as Object3D, e instanceof Error ? e : new Error(String(e)))

// 확장자별 메시 로더 (UR5 visual = .dae / collision = .stl)
function loadMesh(path: string, manager: LoadingManager, done: MeshDone) {
  const ext = path.split('.').pop()?.toLowerCase()
  if (ext === 'dae') {
    new ColladaLoader(manager).load(path, (dae) => done(dae.scene), undefined, (e) => fail(done, e))
  } else if (ext === 'stl') {
    new STLLoader(manager).load(
      path,
      (geom) => done(new Mesh(geom, new MeshPhongMaterial({ color: 0x9aa6b2 }))),
      undefined,
      (e) => fail(done, e),
    )
  } else {
    fail(done, new Error(`unsupported mesh: ${ext}`))
  }
}

interface Props {
  joints: number[]                       // 관절각(rad), joint_names 순서
  onLoaded: (meta: JointMeta[]) => void  // URDF 로드 완료 시 관절 메타 보고
}

/**
 * /api/robot 에서 urdf_url·packages 를 받아 urdf-loader 로 UR5 메시를 로드한다.
 * - URDF 는 Z-up → three(Y-up) 씬에 맞추려 부모에서 -90°(X) 회전해 세운다.
 * - joints prop 이 바뀌면 setJointValues 로 브라우저에서 즉시 FK (백엔드 왕복 0).
 */
export default function RobotView({ joints, onLoaded }: Props) {
  const [robot, setRobot] = useState<URDFRobot | null>(null)
  const [error, setError] = useState<string | null>(null)
  const namesRef = useRef<string[]>([])
  const onLoadedRef = useRef(onLoaded)
  onLoadedRef.current = onLoaded

  // URDF 1회 로드
  useEffect(() => {
    let disposed = false
    getRobot()
      .then((info) => {
        const manager = new LoadingManager()
        const loader = new URDFLoader(manager)
        loader.packages = info.packages
        loader.loadMeshCb = loadMesh
        loader.load(info.urdf_url, (r) => {
          if (disposed) return
          namesRef.current = info.joint_names
          const meta: JointMeta[] = info.joint_names.map((name) => {
            const j = r.joints[name]
            const lo = Number(j?.limit?.lower ?? -Math.PI)
            const hi = Number(j?.limit?.upper ?? Math.PI)
            // continuous 관절(limit 0/0)은 ±π 로 표시
            const continuous = j?.jointType === 'continuous' || (lo === 0 && hi === 0)
            return { name, lower: continuous ? -Math.PI : lo, upper: continuous ? Math.PI : hi }
          })
          setRobot(r)
          onLoadedRef.current(meta)
        })
      })
      .catch((e) => setError(String(e)))
    return () => {
      disposed = true
    }
  }, [])

  // joints 변경 시 URDF 에 적용
  useEffect(() => {
    if (!robot || joints.length === 0) return
    const values: Record<string, number> = {}
    namesRef.current.forEach((name, i) => {
      values[name] = joints[i] ?? 0
    })
    robot.setJointValues(values)
  }, [robot, joints])

  if (error) throw new Error(error)
  if (!robot) return null
  return <primitive object={robot} rotation={[-Math.PI / 2, 0, 0]} />
}
