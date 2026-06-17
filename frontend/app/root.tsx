import { Canvas } from '@react-three/fiber'
import { OrbitControls, Grid } from '@react-three/drei'
import RobotView from '@/components/RobotView'

export default function App() {
  return (
    <div className="relative h-screen w-screen bg-background">
      {/* 헤더 오버레이 (Tailwind 동작 확인 겸) */}
      <header className="pointer-events-none absolute left-0 top-0 z-10 p-4">
        <h1 className="text-lg font-semibold tracking-tight text-foreground">
          UR5 Web Simulator
        </h1>
        <p className="text-sm text-muted-foreground">
          react-three-fiber · urdf-loader · FastAPI
        </p>
      </header>

      <Canvas camera={{ position: [1.4, 1.1, 1.4], fov: 50 }} shadows>
        <ambientLight intensity={0.7} />
        <directionalLight position={[5, 10, 5]} intensity={1.2} castShadow />
        <directionalLight position={[-5, 5, -5]} intensity={0.4} />

        <RobotView />

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
