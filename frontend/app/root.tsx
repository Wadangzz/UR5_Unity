import { useCallback, useState } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls, Grid } from '@react-three/drei'
import RobotView from '@/components/RobotView'
import ControlPanel from '@/components/ControlPanel'
import type { JointMeta } from '@/api'

export default function App() {
  const [meta, setMeta] = useState<JointMeta[]>([])
  const [joints, setJoints] = useState<number[]>([])

  // URDF 로드 완료 → 관절 메타 저장 + 관절각 0 으로 초기화
  const handleLoaded = useCallback((m: JointMeta[]) => {
    setMeta(m)
    setJoints(m.map(() => 0))
  }, [])

  const setJoint = (index: number, value: number) =>
    setJoints((prev) => prev.map((p, i) => (i === index ? value : p)))
  const reset = () => setJoints(meta.map(() => 0))

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
        <ControlPanel meta={meta} joints={joints} onChange={setJoint} onReset={reset} />
      </div>

      <Canvas camera={{ position: [1.4, 1.1, 1.4], fov: 50 }} shadows>
        <ambientLight intensity={0.7} />
        <directionalLight position={[5, 10, 5]} intensity={1.2} castShadow />
        <directionalLight position={[-5, 5, -5]} intensity={0.4} />

        <RobotView joints={joints} onLoaded={handleLoaded} />

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
  )
}
