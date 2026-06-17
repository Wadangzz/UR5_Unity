import { useEffect, useState } from 'react'
import { LoadingManager, Mesh, MeshPhongMaterial, type Object3D } from 'three'
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import URDFLoader from 'urdf-loader'
import type { URDFRobot } from 'urdf-loader'
import { getRobot } from '@/api'

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

/**
 * /api/robot 에서 urdf_url·packages 를 받아 urdf-loader 로 UR5 메시를 로드한다.
 * URDF 는 Z-up → three(Y-up) 씬에 맞추려 부모에서 -90°(X) 회전해 세운다.
 */
export default function RobotView() {
  const [robot, setRobot] = useState<URDFRobot | null>(null)
  const [error, setError] = useState<string | null>(null)

  useEffect(() => {
    let disposed = false
    getRobot()
      .then((info) => {
        const manager = new LoadingManager()
        const loader = new URDFLoader(manager)
        loader.packages = info.packages
        loader.loadMeshCb = loadMesh
        loader.load(info.urdf_url, (r) => {
          if (!disposed) setRobot(r)
        })
      })
      .catch((e) => setError(String(e)))
    return () => {
      disposed = true
    }
  }, [])

  if (error) throw new Error(error)
  if (!robot) return null
  return <primitive object={robot} rotation={[-Math.PI / 2, 0, 0]} />
}
