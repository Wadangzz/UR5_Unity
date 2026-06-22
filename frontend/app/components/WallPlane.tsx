import { useMemo } from 'react';
import { type ThreeEvent } from '@react-three/fiber';
import { DoubleSide, Quaternion, Vector3 } from 'three';

interface Wall {
  shape: string;
  point?: number[];
  normal?: number[];
  center?: number[];
  axis?: number[];
  radius?: number;
}
interface Props {
  wall?: Wall | null;
  onDraw?: (p: number[]) => void; // 표면 클릭 시 경로 점 추가(컨투어 그리기)
}

const COLOR = '#3b82f6';
const OPACITY = 0.22;

/**
 * 힘/하이브리드 제어가 누르는 가상 표면을 Z-up 씬에 렌더한다.
 * plane=point/normal, cylinder=center/axis/radius, sphere=center/radius.
 * (geometry 기본축: plane=+Z, cylinder=+Y → 해당 방향으로 회전시켜 배치)
 */
export default function WallPlane({ wall, onDraw }: Props) {
  const handleClick = onDraw
    ? (e: ThreeEvent<MouseEvent>) => {
        e.stopPropagation();
        onDraw([e.point.x, e.point.y, e.point.z]); // 표면 위 클릭 지점(world)
      }
    : undefined;
  const planeQuat = useMemo(() => {
    if (!wall || wall.shape !== 'plane' || !wall.normal) return null;
    const n = new Vector3(...wall.normal).normalize();
    const q = new Quaternion().setFromUnitVectors(new Vector3(0, 0, 1), n);
    return [q.x, q.y, q.z, q.w] as [number, number, number, number];
  }, [wall]);
  const cylQuat = useMemo(() => {
    if (!wall || wall.shape !== 'cylinder' || !wall.axis) return null;
    const a = new Vector3(...wall.axis).normalize();
    const q = new Quaternion().setFromUnitVectors(new Vector3(0, 1, 0), a);
    return [q.x, q.y, q.z, q.w] as [number, number, number, number];
  }, [wall]);

  if (!wall) return null;
  const r = wall.radius ?? 0.1;

  if (wall.shape === 'cylinder' && wall.center && cylQuat) {
    const [cx, cy, cz] = wall.center;
    return (
      <mesh position={[cx, cy, cz]} quaternion={cylQuat} onClick={handleClick}>
        <cylinderGeometry args={[r, r, 0.5, 48]} />
        <meshStandardMaterial
          color={COLOR}
          transparent
          opacity={OPACITY}
          side={DoubleSide}
        />
      </mesh>
    );
  }
  if (wall.shape === 'sphere' && wall.center) {
    const [cx, cy, cz] = wall.center;
    return (
      <mesh position={[cx, cy, cz]} onClick={handleClick}>
        <sphereGeometry args={[r, 48, 32]} />
        <meshStandardMaterial
          color={COLOR}
          transparent
          opacity={OPACITY}
          side={DoubleSide}
        />
      </mesh>
    );
  }
  if (planeQuat && wall.point) {
    const [px, py, pz] = wall.point;
    return (
      <mesh position={[px, py, pz]} quaternion={planeQuat} onClick={handleClick}>
        <planeGeometry args={[0.6, 0.6]} />
        <meshStandardMaterial
          color={COLOR}
          transparent
          opacity={OPACITY}
          side={DoubleSide}
        />
      </mesh>
    );
  }
  return null;
}
