import { useMemo } from 'react';
import { DoubleSide, Quaternion, Vector3 } from 'three';

interface Props {
  wall?: { point: number[]; normal: number[] } | null;
}

/**
 * 힘/하이브리드 제어가 누르는 가상 벽 평면을 Z-up 씬에 렌더한다.
 * planeGeometry 의 기본 법선은 +Z 이므로, 벽 법선으로 회전시켜 배치한다.
 */
export default function WallPlane({ wall }: Props) {
  const quat = useMemo(() => {
    if (!wall) return null;
    const n = new Vector3(...wall.normal).normalize();
    const q = new Quaternion().setFromUnitVectors(new Vector3(0, 0, 1), n);
    return [q.x, q.y, q.z, q.w] as [number, number, number, number];
  }, [wall]);

  if (!wall || !quat) return null;
  const [px, py, pz] = wall.point;
  return (
    <mesh position={[px, py, pz]} quaternion={quat}>
      <planeGeometry args={[0.6, 0.6]} />
      <meshStandardMaterial
        color='#3b82f6'
        transparent
        opacity={0.22}
        side={DoubleSide}
      />
    </mesh>
  );
}
