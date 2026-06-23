import { useEffect, useRef, useState } from 'react';
import { LoadingManager, Mesh, MeshPhongMaterial, type Object3D } from 'three';
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import URDFLoader from 'urdf-loader';
import type { URDFRobot } from 'urdf-loader';
import { getRobotDetail, type JointMeta } from '@/api';

// urdf-loader 콜백: done(mesh, err) — 에러 시 mesh 자리는 무시되므로 null 을 캐스팅해 넘긴다.
type MeshDone = (mesh: Object3D, err?: Error) => void;
const fail = (done: MeshDone, e: unknown) =>
  done(
    null as unknown as Object3D,
    e instanceof Error ? e : new Error(String(e)),
  );

// iiwa14(.obj)는 재질(.mtl)이 없고 색이 파일명에 인코딩돼 있다(drake 메시 컨벤션):
// link_2_orange.obj / link_2_grey.obj … → 파일명 키워드로 KUKA 색을 입힌다.
// 그 외 .obj 는 기본 회색. (현재 .obj 로봇은 iiwa14 뿐)
function objColor(path: string): number {
  const f = path.toLowerCase();
  if (f.includes('orange')) return 0xe8730c; // KUKA 주황
  if (f.includes('grey') || f.includes('gray')) return 0xd9dde1; // 밝은 회색
  if (f.includes('link_0')) return 0x3f3f46; // 베이스(다크)
  if (f.includes('band')) return 0xc8ccd0; // 실버 밴드
  if (f.includes('kuka')) return 0x27272a; // 로고 플레이트(다크)
  return 0xe8730c; // link_1/3/5/7 = 주황 단일 변형
}

// 확장자별 메시 로더 (ur5/panda visual=.dae·collision=.stl, iiwa14 visual=.obj)
function loadMesh(path: string, manager: LoadingManager, done: MeshDone) {
  const ext = path.split('.').pop()?.toLowerCase();
  if (ext === 'dae') {
    new ColladaLoader(manager).load(
      path,
      (dae) => done(dae.scene),
      undefined,
      (e) => fail(done, e),
    );
  } else if (ext === 'stl') {
    // STL 비주얼은 색 정보가 없어 단색을 입힌다. 등록 로봇 중 omx(OpenManipulator-X)만
    // STL 비주얼을 쓰며 실제 색이 다크 차콜이라, 스틸블루 대신 깔끔한 다크그레이로.
    new STLLoader(manager).load(
      path,
      (geom) =>
        done(
          new Mesh(
            geom,
            // Panda 처럼 깨끗한 화이트 + 은은한 광택
            new MeshPhongMaterial({
              color: 0xf3f4f6,
              specular: 0xffffff,
              shininess: 60,
            }),
          ),
        ),
      undefined,
      (e) => fail(done, e),
    );
  } else if (ext === 'obj') {
    new OBJLoader(manager).load(
      path,
      (obj) => {
        // .obj 는 재질이 없으므로 파일명 기반 색을 phong 으로 입힌다
        const color = objColor(path);
        obj.traverse((c) => {
          const m = c as Mesh;
          if (m.isMesh) m.material = new MeshPhongMaterial({ color });
        });
        done(obj);
      },
      undefined,
      (e) => fail(done, e),
    );
  } else {
    fail(done, new Error(`unsupported mesh: ${ext}`));
  }
}

interface Props {
  robotId: string; // 레지스트리 로봇 id (바뀌면 부모가 key 로 remount)
  joints: number[]; // 관절각(rad), joint_names 순서
  onLoaded: (meta: JointMeta[], ready: number[]) => void; // 관절 메타 + ready 자세 보고
}

/**
 * /api/robots/{id} 에서 urdf_url·packages 를 받아 urdf-loader 로 메시를 로드한다.
 * - 씬을 Z-up(camera.up=+Z)으로 두므로 URDF(Z-up)를 회전 없이 그대로 쓴다
 *   → 월드축 = 로봇 베이스축 = PoseEditor 직교축 일치.
 * - joints prop 이 바뀌면 setJointValues 로 브라우저에서 즉시 FK (백엔드 왕복 0).
 */
export default function RobotView({ robotId, joints, onLoaded }: Props) {
  const [robot, setRobot] = useState<URDFRobot | null>(null);
  const [error, setError] = useState<string | null>(null);
  const namesRef = useRef<string[]>([]);
  const onLoadedRef = useRef(onLoaded);
  onLoadedRef.current = onLoaded;

  // robotId 별 URDF 로드 (바뀌면 이전 로봇 비우고 재로드)
  useEffect(() => {
    let disposed = false;
    setRobot(null);
    setError(null);
    getRobotDetail(robotId)
      .then((info) => {
        const manager = new LoadingManager();
        const loader = new URDFLoader(manager);
        loader.packages = info.packages;
        loader.loadMeshCb = loadMesh;
        loader.load(info.urdf_url, (r) => {
          if (disposed) return;
          namesRef.current = info.joint_names;
          const meta: JointMeta[] = info.joint_names.map((name) => {
            const j = r.joints[name];
            const lo = Number(j?.limit?.lower ?? -Math.PI);
            const hi = Number(j?.limit?.upper ?? Math.PI);
            // continuous 관절(limit 0/0)은 ±π 로 표시
            const continuous =
              j?.jointType === 'continuous' || (lo === 0 && hi === 0);
            return {
              name,
              lower: continuous ? -Math.PI : lo,
              upper: continuous ? Math.PI : hi,
            };
          });
          setRobot(r);
          onLoadedRef.current(meta, info.ready);
        });
      })
      .catch((e) => setError(String(e)));
    return () => {
      disposed = true;
    };
  }, [robotId]);

  // omx(OpenManipulator-X)는 URDF 가 링크에 회색 재질을 박아둬 urdf-loader 가 색을
  // 덮어쓴다 → 칙칙. robot 세팅 후(로드/HMR) 전체 메시를 Panda 풍 화이트로 강제 재색칠.
  useEffect(() => {
    if (!robot || robotId !== 'omx') return;
    // 머리(link5)·그리퍼 = 검정, 나머지 링크 = 흰색. 살짝만 광택.
    // (omx 는 링크 1개=STL 1개라 링크 단위로 색을 입힌다)
    // 늦게 로드되는 STL 까지 잡도록 즉시 + 지연(300ms) 두 번 칠한다.
    const BLACK = new Set(['link5', 'gripper_left_link', 'gripper_right_link']);
    const linkNames = new Set(Object.keys(robot.links));
    const paint = () =>
      robot.traverse((o) => {
        const m = o as Mesh;
        if (!m.material) return;
        let p: Object3D | null = o;
        let link = '';
        while (p) {
          if (linkNames.has(p.name)) {
            link = p.name;
            break;
          }
          p = p.parent;
        }
        m.material = new MeshPhongMaterial({
          color: BLACK.has(link) ? 0x1c1c1f : 0xf3f4f6,
          specular: 0x33373d, // 살짝만 광택
          shininess: 18,
        });
      });
    paint();
    const t = setTimeout(paint, 300);
    return () => clearTimeout(t);
  }, [robot, robotId]);

  // joints 변경 시 URDF 에 적용
  useEffect(() => {
    if (!robot || joints.length === 0) return;
    const values: Record<string, number> = {};
    namesRef.current.forEach((name, i) => {
      values[name] = joints[i] ?? 0;
    });
    robot.setJointValues(values);
  }, [robot, joints]);

  if (error) throw new Error(error);
  if (!robot) return null;
  return <primitive object={robot} />;
}
