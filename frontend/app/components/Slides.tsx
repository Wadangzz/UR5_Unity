import {
  Component,
  Suspense,
  useCallback,
  useEffect,
  useState,
  type ReactNode,
} from 'react';
import { Link } from 'react-router-dom';
import { Canvas } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import katex from 'katex';
import { Card, CardContent } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import RobotView from '@/components/RobotView';
import mrCover from '@/assets/modern_robotics.png';
import fkImg from '@/assets/fk_image.png';
import type { JointMeta } from '@/api';

/* ── 수식 (KaTeX) ── */
function Tex({ t, block = false }: { t: string; block?: boolean }) {
  const html = katex.renderToString(t, {
    displayMode: block,
    throwOnError: false,
  });
  return (
    <span
      className={block ? 'my-1 block' : ''}
      dangerouslySetInnerHTML={{ __html: html }}
    />
  );
}
function Eq({ t }: { t: string }) {
  return (
    <div className='border-border bg-card my-2 inline-block rounded-lg border border-l-4 border-l-sky-500 px-4 py-2 text-[1.05rem] shadow-lg'>
      <Tex t={t} block />
    </div>
  );
}
function Ex({ children }: { children: ReactNode }) {
  return (
    <Card className='border-border bg-muted/40 my-2 border-l-4 border-l-sky-500 py-0 shadow-lg'>
      <CardContent className='text-foreground px-4 py-2 text-[1.1rem] leading-snug'>
        {children}
      </CardContent>
    </Card>
  );
}
function Fig({ svg, cap }: { svg: string; cap?: string }) {
  return (
    <div className='text-center'>
      <div
        className='border-border bg-muted/40 inline-block rounded-xl border p-2 shadow-2xl'
        dangerouslySetInnerHTML={{ __html: svg }}
      />
      {cap && <div className='text-muted-foreground mt-1 text-sm'>{cap}</div>}
    </div>
  );
}
const Cols = ({
  l,
  r,
  title,
}: {
  l: ReactNode;
  r: ReactNode;
  title?: ReactNode;
}) => (
  <div className='flex items-center gap-10'>
    <div className='flex-[1.1]'>
      {title && (
        <div className='mb-4'>
          <Title>{title}</Title>
        </div>
      )}
      {l}
    </div>
    <div className='flex-1 text-center'>{r}</div>
  </div>
);
const UL = ({ items }: { items: ReactNode[] }) => (
  <ul className='my-1 list-disc space-y-1.5 pl-5 text-xl leading-snug marker:text-sky-600'>
    {items.map((it, i) => (
      <li key={i}>{it}</li>
    ))}
  </ul>
);
const Title = ({ children }: { children: ReactNode }) => (
  <h2 className='mb-3 flex items-center gap-2 text-3xl font-extrabold tracking-tight'>
    <span className='inline-block h-5 w-2 rounded-sm bg-sky-500' />
    {children}
  </h2>
);
const A = ({ children }: { children: ReactNode }) => (
  <span className='text-sky-600'>{children}</span>
);
const Num = ({ items }: { items: string[] }) => (
  <div className='mt-2 flex flex-wrap gap-2 text-base'>
    {items.map((x, i) => (
      <span
        key={i}
        className='border-border bg-card rounded-lg border px-3 py-1'
      >
        {x}
      </span>
    ))}
  </div>
);
/* 의사코드 박스 (모노스페이스) + 주석 */
const C = ({ children }: { children: ReactNode }) => (
  <span className='text-muted-foreground'>{children}</span>
);
const Algo = ({ lines }: { lines: ReactNode[] }) => (
  <div className='border-border bg-card my-2 rounded-lg border border-l-4 border-l-emerald-500 px-4 py-2.5 font-mono text-[0.98rem] leading-relaxed whitespace-pre-wrap shadow-lg'>
    {lines.map((l, i) => (
      <div key={i}>{l}</div>
    ))}
  </div>
);

/* ── 실제 로봇 3D 라이브 렌더 (프로젝트 URDF 메시 — 외부 이미지 불필요) ── */
class Boundary extends Component<
  { fallback: ReactNode; children: ReactNode },
  { failed: boolean }
> {
  state = { failed: false };
  static getDerivedStateFromError() {
    return { failed: true };
  }
  render() {
    return this.state.failed ? this.props.fallback : this.props.children;
  }
}

function RobotFigure({
  robotId,
  cap,
  height = 300,
  target = 0.42,
  dist = 1.7,
  capClassName = 'text-sm',
}: {
  robotId: string;
  cap?: string;
  height?: number;
  target?: number;
  dist?: number;
  capClassName?: string;
}) {
  const [joints, setJoints] = useState<number[]>([]);
  const onLoaded = useCallback((_m: JointMeta[], ready: number[]) => {
    setJoints(ready);
  }, []);
  return (
    <div className='text-center'>
      <div
        className='border-border bg-muted/30 mx-auto block overflow-hidden rounded-xl border shadow-2xl'
        style={{ width: Math.round(height * 0.82), height }}
      >
        <Boundary
          fallback={
            <div className='text-muted-foreground flex h-full items-center justify-center px-4 text-center text-sm'>
              3D 렌더에 백엔드(:8500)가 필요합니다
            </div>
          }
        >
          <Canvas
            flat
            camera={{
              position: [dist, -dist, dist * 0.7],
              up: [0, 0, 1],
              fov: 42,
            }}
          >
            <ambientLight intensity={1.0} />
            <hemisphereLight
              intensity={0.8}
              color='#ffffff'
              groundColor='#777777'
            />
            <directionalLight position={[4, -4, 8]} intensity={1.5} />
            <directionalLight position={[-5, 4, 4]} intensity={0.8} />
            <Suspense fallback={null}>
              <RobotView
                robotId={robotId}
                joints={joints}
                onLoaded={onLoaded}
              />
            </Suspense>
            <OrbitControls
              target={[0, 0, target]}
              enableDamping
              autoRotate
              autoRotateSpeed={1.1}
              enablePan={false}
            />
          </Canvas>
        </Boundary>
      </div>
      {cap && (
        <div className={`text-muted-foreground mt-1 ${capClassName}`}>
          {cap}
        </div>
      )}
    </div>
  );
}

/* ── SVG 도식 (raw) ── */
const SCREW = `<svg viewBox="0 0 190 185" width="210"><line x1="95" y1="172" x2="95" y2="20" stroke="#64748b" stroke-width="2" stroke-dasharray="4 4"/><text x="102" y="28" fill="#64748b" font-size="12">축 S</text><path d="M65,148 C65,134 125,134 125,118 C125,102 65,102 65,86 C65,70 125,70 125,54 C125,40 65,40 65,28" fill="none" stroke="#0284c7" stroke-width="3.5"/><line x1="95" y1="166" x2="95" y2="138" stroke="#059669" stroke-width="3" marker-end="url(#ag)"/><text x="100" y="158" fill="#059669" font-size="12">직진</text><path d="M123,116 A29,10 0 0,1 87,116" fill="none" stroke="#d97706" stroke-width="2" marker-end="url(#aw)"/><text x="127" y="110" fill="#d97706" font-size="12">회전</text></svg>`;
const MANIP = `<svg viewBox="0 0 340 180" width="330"><ellipse cx="138" cy="98" rx="28" ry="18" transform="rotate(20 138 98)" fill="rgba(52,211,153,.18)" stroke="#059669" stroke-width="2"/><line x1="35" y1="140" x2="78" y2="72" stroke="#64748b" stroke-width="6"/><line x1="78" y1="72" x2="138" y2="98" stroke="#64748b" stroke-width="6"/><rect x="29" y="137" width="12" height="8" fill="#27344d"/><circle cx="78" cy="72" r="4.5" fill="#3b82f6"/><circle cx="138" cy="98" r="4.5" fill="#d97706"/><text x="95" y="166" fill="#059669" font-size="12.5" text-anchor="middle">정상 — 둥근</text><ellipse cx="291" cy="28" rx="7" ry="25" transform="rotate(-53 291 28)" fill="rgba(251,191,36,.18)" stroke="#d97706" stroke-width="2"/><line x1="205" y1="140" x2="248" y2="84" stroke="#64748b" stroke-width="6"/><line x1="248" y1="84" x2="291" y2="28" stroke="#64748b" stroke-width="6"/><rect x="199" y="137" width="12" height="8" fill="#27344d"/><circle cx="248" cy="84" r="4.5" fill="#3b82f6"/><circle cx="291" cy="28" r="4.5" fill="#d97706"/><text x="252" y="166" fill="#d97706" font-size="12.5" text-anchor="middle">특이점 — 납작</text></svg>`;
const DAMP = `<svg viewBox="0 0 240 150" width="290"><line x1="30" y1="115" x2="225" y2="115" stroke="#27344d"/><line x1="30" y1="25" x2="30" y2="115" stroke="#27344d"/><line x1="30" y1="52" x2="225" y2="52" stroke="#64748b" stroke-dasharray="4 4"/><text x="200" y="47" fill="#64748b" font-size="11">목표</text><path d="M30,115 C70,8 95,72 120,55 C140,45 170,53 225,52" fill="none" stroke="#d97706" stroke-width="2.5"/><path d="M30,115 C80,52 130,52 225,52" fill="none" stroke="#059669" stroke-width="2.5"/><path d="M30,115 C110,92 170,57 225,53" fill="none" stroke="#0284c7" stroke-width="2.5"/><text x="118" y="18" fill="#d97706" font-size="11">ζ&lt;1</text><text x="150" y="42" fill="#059669" font-size="11">ζ=1</text><text x="162" y="78" fill="#0284c7" font-size="11">ζ&gt;1</text></svg>`;
const MSD = `<svg viewBox="0 0 200 165" width="210"><line x1="20" y1="28" x2="180" y2="28" stroke="#64748b" stroke-width="3"/><path d="M100,28 l-10,12 l20,12 l-20,12 l20,12 l-10,12" fill="none" stroke="#0284c7" stroke-width="2.5"/><line x1="130" y1="28" x2="130" y2="76" stroke="#059669" stroke-width="2.5"/><rect x="124" y="58" width="12" height="18" fill="none" stroke="#059669" stroke-width="2"/><rect x="78" y="92" width="48" height="32" rx="5" fill="rgba(59,130,246,.2)" stroke="#3b82f6" stroke-width="2"/><text x="102" y="113" fill="#0f172a" font-size="13" text-anchor="middle">m</text><text x="60" y="58" fill="#0284c7" font-size="13">K</text><text x="142" y="56" fill="#059669" font-size="13">B</text><line x1="102" y1="124" x2="102" y2="150" stroke="#d97706" stroke-width="2.5" marker-end="url(#aw)"/></svg>`;
const CHALK = `<svg viewBox="0 0 220 175" width="240"><rect x="150" y="20" width="22" height="145" fill="rgba(56,189,248,.12)" stroke="#0284c7" stroke-width="2"/><text x="161" y="14" fill="#0284c7" font-size="12" text-anchor="middle">벽</text><line x1="60" y1="92" x2="148" y2="92" stroke="#64748b" stroke-width="6"/><circle cx="148" cy="92" r="6" fill="#d97706"/><line x1="120" y1="92" x2="148" y2="92" stroke="#059669" stroke-width="3" marker-end="url(#ag)"/><text x="116" y="83" fill="#059669" font-size="12">법선 힘</text><line x1="148" y1="118" x2="148" y2="52" stroke="#3b82f6" stroke-width="2.5" stroke-dasharray="5 4" marker-end="url(#a)"/><text x="153" y="138" fill="#3b82f6" font-size="12">접선 이동</text></svg>`;
const GRAV = `<svg viewBox="0 0 190 150" width="210"><line x1="40" y1="50" x2="40" y2="140" stroke="#27344d" stroke-width="6"/><line x1="40" y1="50" x2="145" y2="50" stroke="#64748b" stroke-width="7"/><circle cx="40" cy="50" r="7" fill="#3b82f6"/><line x1="92" y1="58" x2="92" y2="100" stroke="#d97706" stroke-width="2.5" marker-end="url(#aw)"/><line x1="145" y1="58" x2="145" y2="100" stroke="#d97706" stroke-width="2.5" marker-end="url(#aw)"/><text x="112" y="118" fill="#d97706" font-size="12">중력</text><path d="M30,65 A18,18 0 0,1 28,42" fill="none" stroke="#059669" stroke-width="2" marker-end="url(#ag)"/><text x="6" y="48" fill="#059669" font-size="13">τ</text></svg>`;
const JOINTS = `<svg viewBox="0 0 330 150" width="330"><line x1="40" y1="120" x2="40" y2="62" stroke="#64748b" stroke-width="7"/><line x1="40" y1="62" x2="108" y2="42" stroke="#64748b" stroke-width="7"/><circle cx="40" cy="62" r="7" fill="#0284c7"/><path d="M60,44 A24,24 0 0,1 66,72" fill="none" stroke="#d97706" stroke-width="2.5" marker-end="url(#aw)"/><text x="58" y="138" fill="#0284c7" font-size="13" text-anchor="middle">회전관절 (R)</text><rect x="205" y="50" width="74" height="22" rx="3" fill="none" stroke="#64748b" stroke-width="3"/><rect x="218" y="55" width="28" height="12" rx="2" fill="rgba(59,130,246,.28)" stroke="#3b82f6" stroke-width="2"/><line x1="250" y1="61" x2="292" y2="61" stroke="#059669" stroke-width="2.5" marker-end="url(#ag)"/><text x="245" y="138" fill="#059669" font-size="13" text-anchor="middle">직동관절 (P)</text></svg>`;
const ARM6 = `<svg viewBox="0 0 200 178" width="195"><rect x="58" y="160" width="44" height="11" rx="2" fill="#27344d"/><line x1="80" y1="160" x2="80" y2="120" stroke="#64748b" stroke-width="9"/><line x1="80" y1="120" x2="132" y2="80" stroke="#64748b" stroke-width="8"/><line x1="132" y1="80" x2="120" y2="40" stroke="#64748b" stroke-width="7"/><line x1="120" y1="40" x2="152" y2="28" stroke="#64748b" stroke-width="6"/><circle cx="80" cy="120" r="6" fill="#0284c7"/><circle cx="132" cy="80" r="6" fill="#0284c7"/><circle cx="120" cy="40" r="5.5" fill="#0284c7"/><circle cx="152" cy="28" r="5" fill="#d97706"/><text x="160" y="24" fill="#d97706" font-size="11">손끝</text></svg>`;
const IK2 = `<svg viewBox="0 0 240 180" width="245"><rect x="34" y="150" width="14" height="9" fill="#27344d"/><circle cx="182" cy="68" r="6" fill="#d97706"/><text x="190" y="64" fill="#d97706" font-size="12">목표</text><line x1="41" y1="150" x2="148" y2="58" stroke="#0284c7" stroke-width="5"/><line x1="148" y1="58" x2="182" y2="68" stroke="#0284c7" stroke-width="5"/><circle cx="148" cy="58" r="5" fill="#3b82f6"/><line x1="41" y1="150" x2="118" y2="138" stroke="#059669" stroke-width="4" stroke-dasharray="6 4"/><line x1="118" y1="138" x2="182" y2="68" stroke="#059669" stroke-width="4" stroke-dasharray="6 4"/><circle cx="118" cy="138" r="5" fill="#059669"/><text x="58" y="98" fill="#0284c7" font-size="12">해1 (팔꿈치 위)</text><text x="92" y="166" fill="#059669" font-size="12">해2 (팔꿈치 아래)</text></svg>`;
const QUINTIC = `<svg viewBox="0 0 230 160" width="270"><line x1="28" y1="130" x2="214" y2="130" stroke="#27344d"/><line x1="28" y1="22" x2="28" y2="130" stroke="#27344d"/><path d="M28,128 C95,128 110,40 186,40 L210,40" fill="none" stroke="#0284c7" stroke-width="2.5"/><path d="M28,128 Q107,46 186,128" fill="none" stroke="#d97706" stroke-width="2.5"/><text x="138" y="36" fill="#0284c7" font-size="11">위치 s(t)</text><text x="86" y="70" fill="#d97706" font-size="11">속도 ṡ(t)</text><text x="14" y="22" fill="#64748b" font-size="11">1</text><text x="183" y="145" fill="#64748b" font-size="11">T</text><text x="24" y="145" fill="#64748b" font-size="11">0</text></svg>`;

type S = {
  kicker?: string;
  title?: ReactNode;
  body: ReactNode;
  divider?: boolean;
};

const SLIDES: S[] = [
  /* ───────── 0. 오프닝 ───────── */
  {
    body: (
      <div className='text-center'>
        <div className='text-muted-foreground text-2xl'>로봇 웹 시뮬레이터</div>
        <h1 className='my-3 text-5xl leading-tight font-extrabold'>
          로봇은 어떻게 움직이는가
        </h1>
        <div className='text-muted-foreground text-xl'>
          관절 · 자유도 · 기구학 · 동역학 · 경로계획 · 제어
        </div>
      </div>
    ),
  },
  {
    body: (
      <div className='flex items-center justify-center gap-10'>
        <div className='flex-[1.1]'>
          <Title>로봇이 움직이려면?</Title>
          <UL
            items={[
              <>
                ① <b>몸</b> — 어떤 <A>관절</A>이 있고 얼마나 <A>자유롭게</A>{' '}
                움직이나 (관절 · 자유도)
              </>,
              <>
                ② <b>어디로</b> — 관절각 ↔ 손끝 위치 변환 (<A>기구학</A>)
              </>,
              <>
                ③ <b>어떤 힘으로</b> — 중력·관성을 이기는 토크 (<A>동역학</A>)
              </>,
              <>
                ④ <b>어떻게 가서</b> — 부드러운 길 (<A>경로계획</A>)
              </>,
              <>
                ⑤ <b>정확히</b> — 외란이 있어도 목표에 수렴 (<A>제어</A>)
              </>,
            ]}
          />
          <Ex>이 다섯 단계를 차례로 — 마지막엔 실제 시뮬레이터 시연</Ex>
        </div>
        <div className='flex-1 text-center'>
          <img
            src={mrCover}
            alt='Modern Robotics (Lynch & Park) 표지'
            className='border-border mx-auto max-h-[44vh] rounded-lg border shadow-2xl'
          />
          <div className='text-muted-foreground mt-2 text-sm leading-relaxed'>
            참고 · <b>Modern Robotics</b> (Lynch &amp; Park)
            <br />
            <a
              href='https://youtube.com/@erica-ce6vw'
              target='_blank'
              rel='noreferrer'
              className='text-sky-600 hover:underline'
            >
              한양대 최영진 교수 유튜브
            </a>
          </div>
        </div>
      </div>
    ),
  },

  /* ───────── 1. 관절 ───────── */
  {
    body: (
      <Cols
        title='① 관절 — 로봇 움직임의 최소 단위'
        l={
          <>
            <UL
              items={[
                <>
                  <b>회전관절(R):</b> 축을 중심으로 <A>돈다</A> — 사람
                  팔꿈치·어깨
                </>,
                <>
                  <b>직동관절(P):</b> 축을 따라 <A>곧게 밀린다</A> — 서랍·유압
                  실린더
                </>,
                <>
                  로봇 = 링크(뼈) + 관절(마디)의 사슬. 각 관절엔 <b>모터 1개</b>
                  .
                </>,
              ]}
            />
          </>
        }
        r={
          <div className='flex items-end justify-center gap-4'>
            <RobotFigure robotId='ur5' height={220} cap='UR5 · 회전관절 6개' />
            <div className='text-center'>
              <div
                className='border-border bg-muted/30 flex items-center justify-center rounded-xl border p-2 shadow-2xl'
                style={{ height: 220 }}
                dangerouslySetInnerHTML={{ __html: JOINTS }}
              />
              <div className='text-muted-foreground mt-1 text-sm'>
                기본 1자유도 관절 (R · P)
              </div>
            </div>
          </div>
        }
      />
    ),
  },
  {
    body: (
      <Cols
        title='관절을 한 언어로 — 스크류(Screw)'
        l={
          <>
            <UL
              items={[
                <>
                  회전관절·직동관절을 <A>한 가지 수학</A>으로 묶으면 코드가
                  단순해진다
                </>,
                <>
                  <b>Chasles 정리:</b> 모든 강체운동 = 한 축으로 회전+직진 ={' '}
                  <b>나사 운동</b>
                </>,
                <>
                  회전=나사 돌기, 직동=나사 직진(피치 ∞) —{' '}
                  <A>둘 다 스크류축 하나</A>
                </>,
              ]}
            />
            <Ex>
              <b>비유:</b> 병뚜껑·드릴 — 돌면서 들어간다. 우리 코드의 모든
              관절은 스크류축 <Tex t='\mathcal{S}=(\omega,v)' /> 로 표현.
            </Ex>
          </>
        }
        r={<Fig svg={SCREW} cap='Screw = 회전 + 직진' />}
      />
    ),
  },

  /* ───────── 2. 자유도 ───────── */
  {
    body: (
      <Cols
        title='② 자유도(DOF) — 몇 방향으로 자유로운가'
        l={
          <>
            <UL
              items={[
                <>
                  <b>자유도:</b> 독립적으로 정할 수 있는 운동 방향의 수 =
                  관절(모터) 수
                </>,
                <>
                  공간 속 한 물체를 <A>완전히</A> 놓으려면 — 위치 3(
                  <Tex t='x,y,z' />) + 자세 3(<A>roll·pitch·yaw</A>) ={' '}
                  <b>6 자유도</b>
                </>,
                <>
                  그래서 “팔”형 로봇의 표준이 <A>6축</A> (어디든·어느 방향으로든
                  도달)
                </>,
              ]}
            />
            <Ex>
              <b>여유(redundant):</b> 6보다 많으면(7축) 같은 손끝 자세를 여러
              모양으로 — 장애물 회피.
            </Ex>
          </>
        }
        r={<Fig svg={ARM6} cap='위치3 (x·y·z) + 자세3 (roll·pitch·yaw)' />}
      />
    ),
  },
  {
    title: '여러 로봇의 자유도',
    body: (
      <>
        <div className='flex items-end justify-center gap-8'>
          <RobotFigure
            robotId='ur5'
            height={360}
            capClassName='text-lg'
            cap='UR5 · 6-DOF'
          />
          <RobotFigure
            robotId='panda'
            height={360}
            capClassName='text-lg'
            cap='Franka Panda · 7-DOF'
          />
          <RobotFigure
            robotId='iiwa14'
            height={360}
            capClassName='text-lg'
            cap='KUKA iiwa14 · 7-DOF'
          />
        </div>
        <div className='text-muted-foreground mt-4 text-center text-xl'>
          셋 다 <b className='text-foreground'>회전관절(R)만</b> — UR5는 R×6,
          Panda·iiwa14는 R×7(<b className='text-foreground'>여유자유도</b>). 그
          외: 2축 <b className='text-foreground'>2</b> · 델타{' '}
          <b className='text-foreground'>3</b> · SCARA{' '}
          <b className='text-foreground'>4</b> · 산업용 표준{' '}
          <b className='text-foreground'>6</b> — 자유도가 도달 범위와 IK
          난이도를 결정
        </div>
      </>
    ),
  },

  /* ───────── 3. 기구학 ───────── */
  {
    title: '③ 기구학 — 왜 필요한가',
    body: (
      <>
        <UL
          items={[
            <>
              모터가 아는 건 <b>관절각</b> 뿐. 우리가 원하는 건{' '}
              <b>손끝의 위치·자세</b>.
            </>,
            <>
              둘 사이를 오가는 <A>번역기</A>가 기구학 — 없으면 “여기 잡아”를
              모터 명령으로 바꿀 수 없다.
            </>,
          ]}
        />
        <Eq t='\underbrace{(\theta_1,\dots,\theta_6)}_{\text{관절각}}\;\xrightarrow{\ \text{FK}\ }\;\underbrace{T=(R,p)}_{\text{손끝 자세}}\;\xrightarrow{\ \text{IK}\ }\;(\theta_1,\dots,\theta_6)' />
      </>
    ),
  },
  {
    body: (
      <Cols
        title='정기구학(FK) — 관절각 → 손끝'
        l={
          <>
            <UL
              items={[
                <>
                  관절각을 주면 손끝 위치는 <A>계산만 하면 유일하게</A> 나온다
                  (쉬움)
                </>,
                <>
                  각 관절의 스크류 운동을 차례로 곱한다 — <b>PoE(지수곱)</b>
                </>,
              ]}
            />
            <Eq t='T(\theta)=e^{[\mathcal{S}_1]\theta_1}\,e^{[\mathcal{S}_2]\theta_2}\cdots e^{[\mathcal{S}_6]\theta_6}\,M' />
          </>
        }
        r={
          <Fig
            svg={`<img src="${fkImg}" alt="PoE 지수곱" style="max-height:46vh;border-radius:8px" />`}
            cap='PoE — 스크류 지수곱을 차례로 (MR)'
          />
        }
      />
    ),
  },
  {
    body: (
      <Cols
        title='역기구학(IK) — 손끝 → 관절각 (어려움)'
        l={
          <>
            <UL
              items={[
                <>
                  “여기 잡아”(목표 자세) → 관절각을 <A>거꾸로</A> 풀어야 한다
                </>,
                <>
                  <b>해가 여러 개:</b> 2축 팔도 한 점에 <A>팔꿈치 위/아래</A> 두
                  해
                </>,
                <>
                  도달 못 하는 곳(작업영역 밖)은 <b>해가 0개</b>이기도
                </>,
              ]}
            />
            <Ex>
              그래서 IK는 “계산”이 아니라 <b>방정식 풀이</b> 문제.
            </Ex>
          </>
        }
        r={<Fig svg={IK2} cap='2축 → 한 점에 해 2개' />}
      />
    ),
  },
  {
    title: '축이 늘면 — 해석해가 어려워진다',
    body: (
      <>
        <UL
          items={[
            <>
              2축은 삼각함수로 손풀이. 그러나 <A>일반 6축</A>은 닫힌형 공식이
              없는 게 보통이다.
            </>,
            <>
              비선형 방정식 6개가 얽힘 → <b>수치해석(반복 근사)</b>로 푼다
            </>,
            <>
              한 가지 수치 IK로 <A>UR5 · Panda · iiwa(7축 여유)</A>까지 모두 —
              로봇 무관 + 특이점 강건(DLS)
            </>,
          ]}
        />
        <Ex>
          <b>예외:</b> 손목 3축이 한 점에 교차(스피리컬) 또는 <A>축이 평행</A>
          하면 닫힌해 존재. <b>UR은 관절 2·3·4가 평행</b>해 해석해가 있다 —
          Hawkins(2013), 최대 8개 해.
        </Ex>
      </>
    ),
  },
  {
    body: (
      <Cols
        title='Newton-Raphson + DLS'
        l={
          <>
            <UL
              items={[
                <>
                  <b>Newton-Raphson:</b> 자코비안 <Tex t='J_b' /> 로 “오차 →
                  관절보정”을 반복
                </>,
                <>
                  보통 <A>수 회 반복에 0.1mm 이하</A> 수렴 (코드:{' '}
                  <Tex t='10^{-4}' /> 허용오차)
                </>,
                <>
                  <b>특이점 문제:</b> 팔을 쭉 펴면 <Tex t='J_b' /> 가 무너져
                  보정량 폭주
                </>,
                <>
                  → <A>DLS</A>(감쇠최소제곱)로 <Tex t='\sigma_{\min}<0.04' /> 일
                  때 부드럽게 감쇠
                </>,
              ]}
            />
            <Eq t='\theta \leftarrow \theta + J_b^{\top}(J_bJ_b^{\top}+\lambda^2 I)^{-1}\,\mathcal{V}_b' />
          </>
        }
        r={<Fig svg={MANIP} cap='특이점=조작성 타원 납작' />}
      />
    ),
  },

  /* ───────── 4. 동역학 ───────── */
  {
    title: '④ 동역학 — 왜 필요한가',
    body: (
      <>
        <UL
          items={[
            <>“각도만 맞추면 되지” → X</>,
            <>
              팔은 <A>중력</A>에 처지고, <A>관성</A>
              으로 흔들리고, 빨리 돌면 <A>코리올리</A> 힘을 받는다.
            </>,
            <>
              이 힘들을 이기려면 <b>각 모터가 낼 토크</b>를 알아야 한다 → 그게
              동역학.
            </>,
          ]}
        />
        <Eq t='M(\theta)\,\ddot\theta + c(\theta,\dot\theta) + g(\theta) = \tau' />
        <div className='text-muted-foreground text-lg'>
          관성 <Tex t='M' /> · 코리올리/원심 <Tex t='c' /> · 중력 <Tex t='g' />{' '}
          · 모터토크 <Tex t='\tau' />
        </div>
      </>
    ),
  },
  {
    body: (
      <Cols
        title='Recursive Newton-Euler'
        l={
          <>
            <UL
              items={[
                <>
                  동역학 해석 두 방법: <A>라그랑지안</A>(에너지) ·{' '}
                  <A>뉴턴-오일러</A>(힘)
                </>,
                <>
                  같은 결과, 시뮬레이션에는 <b>뉴턴-오일러</b> 적용
                </>,
                <>
                  <b>Newton-Euler(RNE)</b> 로 <Tex t='M,c,g' /> 를 계산
                </>,
                <>
                  <b>시뮬레이터의 심장:</b>{' '}
                  <Tex t='\ddot\theta=M^{-1}(\tau-c-g)' /> 를 적분해 로봇의 진짜
                  운동을 재현
                </>,
                <>
                  중력보상 <Tex t='\tilde g(\theta)' /> 는 제어기에 그대로
                  들어가 팔이 처지지 않게
                </>,
              ]}
            />
          </>
        }
        r={<Fig svg={GRAV} cap='뻗을수록 어깨 토크 ↑' />}
      />
    ),
  },

  /* ───────── 5. 경로계획 ───────── */
  {
    title: '⑤ 경로계획 — 왜 필요한가',
    body: (
      <>
        <UL
          items={[
            <>
              A→B로 <A>갑자기</A> 점프하면? 무한 가속 → 무한 토크 → 충격·진동.
            </>,
            <>
              관절이 <b>천천히 출발해 천천히 멈추는</b> 부드러운 시간함수가
              필요.
            </>,
            <>
              경로(어디를 지나갈지) + <A>타임스케일링</A>(언제 얼마나 빨리) 로
              분리.
            </>,
          ]}
        />
        <Ex>
          목표: 시작·끝에서 <b>속도 0, 가속도 0</b> — 덜컹임 없이 출발/정지.
        </Ex>
      </>
    ),
  },
  {
    body: (
      <Cols
        title='5차 다항식 타임스케일링'
        l={
          <>
            <UL
              items={[
                <>
                  <Tex t='s(t)' /> 가 0→1 로 매끄럽게 — 경계조건 6개를{' '}
                  <b>5차 다항식</b>으로 만족
                </>,
                <>
                  <Tex t='s(0)=0,\ s(T)=1' /> 그리고 <Tex t='\dot s,\ddot s' />{' '}
                  가 양 끝에서 <A>0</A>
                </>,
                <>
                  관절 목표:{' '}
                  <Tex t='\theta_d(t)=\theta_0+s(t)\,(\theta_1-\theta_0)' />
                </>,
              ]}
            />
            <Eq t='s(t)=a_3t^3+a_4t^4+a_5t^5' />
            <Ex>
              우리 코드: <Tex t='quintic\_time\_scaling(t,T)' /> 가{' '}
              <Tex t='(s,\dot s,\ddot s)' /> 를 반환 → 구간별 보간.
            </Ex>
          </>
        }
        r={<Fig svg={QUINTIC} cap='위치 S커브 · 속도 종모양' />}
      />
    ),
  },

  /* ───────── 6. 제어 ───────── */
  {
    title: '⑥ 제어 — 왜 필요한가',
    body: (
      <>
        <UL
          items={[
            <>
              경로대로 토크를 줘도 <A>그대로 안 간다</A> — 마찰·모델오차·외란
              때문.
            </>,
            <>
              <b>피드백:</b> 매 순간 “목표 − 현재” 오차를 재서 토크를 보정.
            </>,
          ]}
        />
        <Eq t='\tau = (\text{피드포워드})\;+\;K_p\,e\;+\;K_d\,\dot e\;+\;K_i\!\int e' />
        <div className='text-muted-foreground text-lg'>
          오차 <Tex t='e=\theta_d-\theta' /> — 이걸 0으로 끌고 가는 게 제어기.
        </div>
      </>
    ),
  },
  {
    body: (
      <Cols
        title='제어의 직관 — 스프링처럼 (2차계)'
        l={
          <>
            <UL
              items={[
                <>
                  목표로 당기는 <b>스프링(Kp)</b> + 흔들림 잡는 <b>댐퍼(Kd)</b>
                </>,
                <>
                  감쇠비 <Tex t='\zeta' />: 출렁(&lt;1) · 딱(=1) · 느림(&gt;1)
                </>,
                <>
                  <b>Kp↑</b> 빠르지만 오버슛 / <b>Kd↑</b> 안정하지만 노이즈에
                  민감
                </>,
              ]}
            />
            <Eq t='\ddot e + 2\zeta\omega_n\dot e + \omega_n^2 e = 0' />
            <Ex>
              <b>예:</b> 자동차 서스펜션 — 균형점이 <Tex t='\zeta\approx1' />.
            </Ex>
          </>
        }
        r={<Fig svg={DAMP} cap='감쇠비별 응답' />}
      />
    ),
  },
  {
    title: '관절공간 vs 작업공간 — 왜 나누나',
    body: (
      <>
        <table className='w-full border-collapse text-[1.0rem]'>
          <thead>
            <tr className='bg-card text-sky-600'>
              <th className='border-border border p-2 text-left'>구분</th>
              <th className='border-border border p-2 text-left'>
                관절공간 제어 (오차 = 관절각)
              </th>
              <th className='border-border border p-2 text-left'>
                작업공간 제어 (오차 = 손끝 자세)
              </th>
            </tr>
          </thead>
          <tbody>
            {[
              [
                '제어기',
                'PD · PID · Computed Torque · 관절 속도제어',
                '작업공간 속도제어 · 임피던스 · 어드미턴스 · 힘 · 하이브리드',
              ],
              ['오차를 재는 곳', '관절각 (rad)', '손끝 위치·자세 (m, rad)'],
              ['장점', '단순·빠름·항상 실행가능', '말단 직선/자세를 직접 지정'],
              ['말단 경로', '곡선 (예측 어려움)', '직선·원호 등 의도대로'],
              ['필요한 것', '관절 목표만', '자코비안 J† (특이점 위험)'],
              ['쓰는 때', '점-대-점 이동·홈복귀', '용접·도포·조립 등 경로작업'],
            ].map((r, i) => (
              <tr key={i} className={i === 0 ? 'bg-sky-500/10' : ''}>
                <td
                  className={`border-border border p-1.5 ${i === 0 ? 'text-foreground font-semibold' : 'text-muted-foreground'}`}
                >
                  {r[0]}
                </td>
                <td className='border-border border p-1.5'>{r[1]}</td>
                <td className='border-border border p-1.5'>{r[2]}</td>
              </tr>
            ))}
          </tbody>
        </table>
        <div className='text-muted-foreground mt-2 text-base'>
          ※ <b className='text-foreground'>Computed Torque</b>(역동역학)는
          작업공간으로도 확장 가능
        </div>
      </>
    ),
  },
  {
    title: '관절공간 제어기 — 3종',
    body: (
      <>
        <UL
          items={[
            <>
              <b>PD + 중력보상</b> <Tex t='\tau=K_pe+K_d\dot e+\tilde g' /> —
              가장 단순, 잔류오차 약간
            </>,
            <>
              <b>PID</b> — 적분(I)이 <A>마찰·중력 잔류오차</A>를 제거 (단,
              과하면 발산)
            </>,
            <>
              <b>Computed Torque</b> — 비선형 <Tex t='M,c,g' /> 를 상쇄해 완전한
              선형계로 (가장 강건)
            </>,
          ]}
        />
        <Eq t='\tau=M(\theta)\big(\ddot\theta_d+K_p e+K_d\dot e+K_i\!\textstyle\int e\big)+c+g' />
      </>
    ),
  },
  {
    title: '파라미터를 바꾸면? — 실측 반응',
    body: (
      <>
        <table className='w-full border-collapse text-[1.0rem]'>
          <thead>
            <tr className='bg-card text-sky-600'>
              <th className='border-border border p-2 text-left'>Pamameter</th>
              <th className='border-border border p-2 text-left'>
                관찰되는 반응
              </th>
            </tr>
          </thead>
          <tbody>
            {[
              ['Kp ↑', '빠르게 도달하지만 오버슛·진동 ↑'],
              ['Kd ↑', '진동 잡힘 — 그러나 센서 노이즈에 토크 chatter ↑'],
              ['Ki ↑', '잔류오차 제거 — Ki > Kp·Kd 넘으면 발산 (Routh)'],
              [
                '제어율 ↓ (1000→500Hz)',
                '게인 그대로인데 발산 — sim-to-real 절벽',
              ],
              [
                '자동튜닝 ts ↓ (0.3→0.2s)',
                'Kp 폭증(178→400) → 더 빠른 제어보드 요구',
              ],
            ].map((r, i) => (
              <tr key={i}>
                <td className='border-border border p-1.5 font-semibold'>
                  {r[0]}
                </td>
                <td className='border-border text-muted-foreground border p-1.5'>
                  {r[1]}
                </td>
              </tr>
            ))}
          </tbody>
        </table>
        <Ex>
          <b>핵심:</b> 발산은 버그가 아니라 “이 목표면 제어보드가 몇 Hz는 돼야
          한다”는 신호.
        </Ex>
      </>
    ),
  },
  {
    title: '작업공간 속도제어',
    body: (
      <>
        <UL
          items={[
            <>
              손끝 속도(트위스트)를 직접 명령 → 자코비안 역으로 관절속도로 변환
            </>,
            <>
              <A>궤적 ⊥ 제어기</A>: 직교 직선궤적을 주면 말단이 <b>직선</b>으로
              추종 (이탈 0.0mm)
            </>,
          ]}
        />
        <Eq t='\mathcal{V}_b=[\mathrm{Ad}]\mathcal{V}_d+K_pX_e,\qquad \dot\theta=J_b^{\dagger}\mathcal{V}_b' />
        <div className='text-muted-foreground text-lg'>
          파라미터 <Tex t='K_p' /> ↑ → 직선 추종 빨라짐. 시작이 특이점이면 “실행
          불가”로 정직하게 표면화.
        </div>
      </>
    ),
  },
  {
    body: (
      <Cols
        title='임피던스 ↔ 어드미턴스 — 환경과의 대화'
        l={
          <>
            <UL
              items={[
                <>
                  <b>임피던스</b>: 움직임을 보고 힘을 낸다 (가상 스프링) — 밀면
                  밀림
                </>,
                <>
                  <b>어드미턴스</b>: 힘을 느끼고 움직인다 — 능동적으로 양보
                </>,
                <>
                  파라미터 <Tex t='K' /> ↑ → 단단(밀림 ½) / <Tex t='K' /> ↓ →
                  말랑(밀림 2배)
                </>,
              ]}
            />
            <Ex>
              <b>예:</b> 악수=임피던스, 무거운 문 밀기=어드미턴스.
            </Ex>
          </>
        }
        r={<Fig svg={MSD} cap='가상 질량-스프링-댐퍼' />}
      />
    ),
  },
  {
    body: (
      <Cols
        title='힘 & 하이브리드 제어 — 표면 작업'
        l={
          <>
            <UL
              items={[
                <>
                  <b>힘 제어:</b> 벽을 <A>목표 힘(20N)</A>으로 일정하게 누름
                </>,
                <>
                  <b>하이브리드:</b>{' '}
                  <span className='text-emerald-600'>법선=힘 제어</span> ·{' '}
                  <A>접선=위치 제어</A> 동시 (칠판에 분필로 선 긋기)
                </>,
                <>
                  곡면(평면/원기둥/구/메시) <b>컨투어 추종</b>까지 — 법선이 매
                  순간 바뀌어도 동일 식
                </>,
              ]}
            />
            <Num items={['힘오차 ≈ 0%', '접선오차 0.8mm']} />
          </>
        }
        r={<Fig svg={CHALK} cap='누르며 선 긋기' />}
      />
    ),
  },

  /* ───────── 7. 연동 · 시연 ───────── */
  {
    title: '실무 연동 — ROS 2 · Isaac Sim',
    body: (
      <table className='w-full border-collapse text-[1.02rem]'>
        <thead>
          <tr className='bg-card text-sky-600'>
            <th className='border-border border p-2 text-left'>기능</th>
            <th className='border-border border p-2 text-left'>ROS 2</th>
            <th className='border-border border p-2 text-left'>Isaac Sim</th>
          </tr>
        </thead>
        <tbody>
          {[
            ['FK / IK', 'MoveIt (KDL·TracIK)', 'Lula Kinematics'],
            [
              '관절공간',
              'joint_trajectory_controller',
              'ArticulationController (PD drive)',
            ],
            [
              '작업공간 직선',
              'MoveIt computeCartesianPath',
              'RmpFlow · LulaTrajectory',
            ],
            ['작업공간 속도제어', 'moveit_servo (Twist)', 'RmpFlow · Jacobian'],
            ['동역학 M,c,g', 'ros2_control (effort)', 'physics tensor API'],
            [
              '임피던스/어드미턴스',
              'cartesian_compliance / admittance',
              'joint drive stiffness·damping',
            ],
            [
              '힘 / 하이브리드',
              'cartesian_force_controller',
              'effort + PhysX contact',
            ],
          ].map((r, i) => (
            <tr key={i}>
              <td className='border-border text-muted-foreground border p-1.5'>
                {r[0]}
              </td>
              <td className='border-border border p-1.5'>{r[1]}</td>
              <td className='border-border border p-1.5'>{r[2]}</td>
            </tr>
          ))}
        </tbody>
      </table>
    ),
  },
  {
    title: '라이브 데모 & 정리',
    body: (
      <div className='text-center'>
        <Link to='/'>
          <Button size='lg' className='my-3 text-lg'>
            ▶ 시뮬레이터 열기
          </Button>
        </Link>
        <div className='mt-2 flex flex-wrap justify-center gap-2 text-base'>
          {[
            '관절·자유도 → 몸',
            'FK/IK → 어디로',
            '동역학 → 어떤 힘으로',
            '5차 경로 → 어떻게',
            'PD·PID·CT·임피던스·힘 → 정확히',
          ].map((t, i) => (
            <span
              key={i}
              className='rounded-full border border-sky-500 bg-sky-500/15 px-3 py-1 text-sky-700'
            >
              {t}
            </span>
          ))}
        </div>
        <div className='text-muted-foreground mt-4'>
          감사합니다 🙇 — 수학적 배경은 부록에
        </div>
      </div>
    ),
  },

  /* ───────── 부록 ───────── */
  {
    divider: true,
    kicker: 'APPENDIX · 부록',
    title: '수학적 배경',
    body: (
      <div className='text-muted-foreground text-xl'>
        Modern Robotics — Lie 군 · 스크류 · 동역학 · 제어 법칙
      </div>
    ),
  },
  {
    title: 'SO(3) Lie Group & so(3) Algebra',
    body: (
      <>
        <UL
          items={[
            <>
              회전군 <Tex t='SO(3)=\{R\mid R^\top R=I,\ \det R=1\}' />
            </>,
            <>
              <Tex t='R^\top R=I' /> 미분 → 각속도 반대칭:
            </>,
          ]}
        />
        <Eq t='[\omega]=R^\top\dot R=-[\omega]^\top\in so(3),\quad R=\exp([\omega]\theta)' />
      </>
    ),
  },
  {
    title: 'Exponential Coordinates & Rodrigues',
    body: (
      <>
        <Eq t='e^{[\hat\omega]\theta}=I+\sin\theta\,[\hat\omega]+(1-\cos\theta)[\hat\omega]^2' />
        <Eq t='\theta=\cos^{-1}\!\tfrac{\operatorname{tr}R-1}{2},\quad [\hat\omega]=\tfrac{R-R^\top}{2\sin\theta}' />
        <Ex>
          각속도 ODE <Tex t='\dot R=[\omega]R' />의 해가 행렬지수 — Rodrigues는
          그 닫힌식.
        </Ex>
      </>
    ),
  },
  {
    title: 'SE(3) · Adjoint · Screw',
    body: (
      <>
        <Eq t='T=\begin{bmatrix}R&p\\0&1\end{bmatrix},\quad [\mathrm{Ad}_T]=\begin{bmatrix}R&0\\ [p]R&R\end{bmatrix}' />
        <Eq t='e^{[\mathcal{S}]\theta}=\begin{bmatrix}e^{[\omega]\theta}&G(\theta)v\\0&1\end{bmatrix},\ G=I\theta+(1-\cos\theta)[\omega]+(\theta-\sin\theta)[\omega]^2' />
      </>
    ),
  },
  {
    body: (
      <Cols
        title='Body Jacobian & Singularity'
        l={
          <>
            <Eq t='\mathcal{V}_b=J_b(\theta)\dot\theta' />
            <Eq t='w=\sqrt{\det(J_bJ_b^\top)},\ \dot\theta=J_b^\top(J_bJ_b^\top+\lambda^2 I)^{-1}\mathcal{V}_b' />
            <UL
              items={[
                <>
                  <Tex t='w\to0' /> = 특이점 → 타원 납작
                </>,
              ]}
            />
          </>
        }
        r={<Fig svg={MANIP} cap='조작성 타원' />}
      />
    ),
  },
  {
    title: 'PoE 정기구학 & IK = 근찾기 문제',
    body: (
      <>
        <Eq t='T(\theta)=e^{[\mathcal{S}_1]\theta_1}\cdots e^{[\mathcal{S}_n]\theta_n}\,M,\quad \mathcal{S}_i=(\omega_i,\,-\omega_i\times q_i)' />
        <UL
          items={[
            <>
              <b>IK:</b> <Tex t='T(\theta)=T_d' /> 를 푼다 ⇔ 오차함수의 근{' '}
              <Tex t='e(\theta)=\log\!\big(T(\theta)^{-1}T_d\big)=0' />
            </>,
            <>
              비선형 6식 → 닫힌해 일반적으로 없음 → <A>뉴턴-랩슨 반복</A>(다음
              장)
            </>,
            <>
              오차의 1차 민감도 = <b>body Jacobian</b>{' '}
              <Tex t='\delta e \approx J_b(\theta)\,\delta\theta' />
            </>,
          ]}
        />
      </>
    ),
  },
  {
    body: (
      <Cols
        title='수치 IK — Newton-Raphson 알고리즘'
        l={
          <>
            <Algo
              lines={[
                <>
                  θ ← θ₀ <C>{'// 초기 추정(seed)'}</C>
                </>,
                <>repeat (k = 0,1,2,…):</>,
                <>
                  {'  '}T_e = T(θ)⁻¹·T_d <C>{'// 현재→목표 오차'}</C>
                </>,
                <>
                  {'  '}𝒱_b = log(T_e) = [ω_b, v_b] <C>{'// body twist'}</C>
                </>,
                <>
                  {'  '}if ‖ω_b‖{'<'}ε_ω and ‖v_b‖{'<'}ε_v: return θ
                </>,
                <>
                  {'  '}θ ← θ + J_b⁺(θ)·𝒱_b <C>{'// 뉴턴 보정'}</C>
                </>,
              ]}
            />
            <div className='text-muted-foreground text-base'>
              코드: <Tex t='\varepsilon=10^{-4}' />, max_iter 100, 해마다{' '}
              <Tex t='[-\pi,\pi]' /> wrap
            </div>
          </>
        }
        r={
          <>
            <Eq t='J_b^{\dagger}=J_b^\top\big(J_bJ_b^\top+\lambda^2 I\big)^{-1}\ (\sigma_{\min}<\varepsilon)' />
            <Ex>
              해 근처 <b>2차 수렴</b>(보통 3~10회). 특이점선 <Tex t='J_b' />{' '}
              붕괴 → <A>DLS</A> 감쇠
            </Ex>
          </>
        }
      />
    ),
  },
  {
    title: '라그랑지안 동역학 (Euler-Lagrange)',
    body: (
      <>
        <Eq t='\mathcal{L}(\theta,\dot\theta)=\mathcal{K}-\mathcal{P}=\tfrac12\,\dot\theta^\top M(\theta)\,\dot\theta-\mathcal{P}(\theta)' />
        <Eq t='\frac{d}{dt}\frac{\partial\mathcal{L}}{\partial\dot\theta}-\frac{\partial\mathcal{L}}{\partial\theta}=\tau\;\Rightarrow\; M(\theta)\ddot\theta+C(\theta,\dot\theta)\dot\theta+g(\theta)=\tau' />
        <Eq t='C_{ij}=\sum_k\Gamma_{ijk}\dot\theta_k,\ \ \Gamma_{ijk}=\tfrac12\!\Big(\tfrac{\partial M_{ij}}{\partial\theta_k}+\tfrac{\partial M_{ik}}{\partial\theta_j}-\tfrac{\partial M_{jk}}{\partial\theta_i}\Big),\ \ g=\tfrac{\partial\mathcal{P}}{\partial\theta}' />
        <Ex>
          에너지 <Tex t='\mathcal{K}-\mathcal{P}' /> 를 미분 → 같은{' '}
          <Tex t='M,C,g' />. <b>뉴턴-오일러(RNE)</b> 는 같은 결과를 힘 전파로{' '}
          <Tex t='O(n)' /> 에
        </Ex>
      </>
    ),
  },
  {
    title: 'RNE — 순방향/역방향 2-pass 알고리즘',
    body: (
      <div className='flex flex-col items-start'>
        <div className='text-base font-semibold text-sky-600'>
          ① 순방향 (base→tip): 속도·가속 전파
        </div>
        <Eq t='\mathcal{V}_i=[\mathrm{Ad}_{T_{i,i-1}}]\mathcal{V}_{i-1}+A_i\dot\theta_i' />
        <Eq t='\dot{\mathcal{V}}_i=[\mathrm{Ad}_{T_{i,i-1}}]\dot{\mathcal{V}}_{i-1}+[\mathrm{ad}_{\mathcal{V}_i}]A_i\dot\theta_i+A_i\ddot\theta_i' />
        <div className='mt-2 text-base font-semibold text-emerald-600'>
          ② 역방향 (tip→base): 렌치 전파 → 토크
        </div>
        <Eq t='\mathcal{F}_i=[\mathrm{Ad}_{T_{i+1,i}}]^\top\mathcal{F}_{i+1}+\mathcal{G}_i\dot{\mathcal{V}}_i-[\mathrm{ad}_{\mathcal{V}_i}]^\top\mathcal{G}_i\mathcal{V}_i' />
        <Eq t='\tau_i=\mathcal{F}_i^\top A_i' />
        <div className='text-muted-foreground mt-2 text-base'>
          <b className='text-foreground'>초기조건 트릭:</b>{' '}
          <Tex t='\mathcal{V}_0=0,\ \dot{\mathcal{V}}_0=(0,-g)' /> →{' '}
          <A>중력 자동 포함</A> ·{' '}
          <Tex t='\mathcal{F}_{n+1}=\mathcal{F}_{tip}' /> (외력) · 모두
          6벡터·6×6 곱 → <b className='text-foreground'>O(n)</b>
        </div>
      </div>
    ),
  },
  {
    title: 'RNE에서 M·c·g 추출 & 제어 법칙',
    body: (
      <>
        <Eq t='g(\theta)=\mathrm{RNE}(\theta,0,0),\quad c(\theta,\dot\theta)=\mathrm{RNE}(\theta,\dot\theta,0)\big|_{g=0}' />
        <Eq t='M_{\cdot j}(\theta)=\mathrm{RNE}(\theta,0,e_j)\big|_{g=0},\quad \ddot\theta=M^{-1}(\tau-c-g-J^\top\mathcal{F}_{tip})' />
        <Eq t='\text{힘: }\tau=\tilde g+J_b^\top(F_d+K_{fp}F_e+K_{fi}\!\int F_e-K_d\mathcal{V})' />
        <Eq t='\text{하이브리드: }P=I-A^\top(A\Lambda^{-1}A^\top)^{-1}A\Lambda^{-1}' />
      </>
    ),
  },
];

export default function Slides() {
  const [cur, setCur] = useState(0);
  const n = SLIDES.length;
  const go = useCallback(
    (d: number) => setCur((c) => Math.max(0, Math.min(n - 1, c + d))),
    [n],
  );
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (['ArrowRight', ' ', 'PageDown'].includes(e.key)) {
        e.preventDefault();
        go(1);
      } else if (['ArrowLeft', 'PageUp'].includes(e.key)) {
        e.preventDefault();
        go(-1);
      } else if (e.key === 'Home') setCur(0);
      else if (e.key === 'End') setCur(n - 1);
      else if (e.key === 'f' || e.key === 'F') {
        if (!document.fullscreenElement)
          document.documentElement.requestFullscreen();
        else document.exitFullscreen();
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [go, n]);

  const s = SLIDES[cur];
  return (
    <div className='bg-background text-foreground fixed inset-0 overflow-hidden'>
      {/* 공통 SVG 화살표 마커 */}
      <svg width='0' height='0' className='absolute'>
        <defs>
          <marker
            id='a'
            markerWidth='9'
            markerHeight='9'
            refX='7'
            refY='3'
            orient='auto'
          >
            <path d='M0,0L7,3L0,6Z' fill='#0284c7' />
          </marker>
          <marker
            id='ag'
            markerWidth='9'
            markerHeight='9'
            refX='7'
            refY='3'
            orient='auto'
          >
            <path d='M0,0L7,3L0,6Z' fill='#059669' />
          </marker>
          <marker
            id='aw'
            markerWidth='9'
            markerHeight='9'
            refX='7'
            refY='3'
            orient='auto'
          >
            <path d='M0,0L7,3L0,6Z' fill='#d97706' />
          </marker>
        </defs>
      </svg>

      <div
        className='absolute top-0 left-0 z-10 h-1 bg-gradient-to-r from-blue-500 to-sky-400 transition-all'
        style={{ width: `${((cur + 1) / n) * 100}%` }}
      />

      <div
        key={cur}
        className={`animate-in fade-in flex h-full flex-col justify-center px-[7vw] py-[6vh] duration-300 ${s.divider ? 'items-start bg-[radial-gradient(120%_80%_at_100%_0%,rgba(59,130,246,.18),transparent_60%)]' : ''}`}
      >
        {s.kicker && (
          <div className='text-xl font-bold tracking-[.25em] text-sky-600'>
            {s.kicker}
          </div>
        )}
        {s.title &&
          (s.divider ? (
            <h1 className='my-2 text-5xl font-extrabold'>{s.title}</h1>
          ) : (
            <Title>{s.title}</Title>
          ))}
        {s.body}
      </div>

      <div className='text-muted-foreground absolute right-0 bottom-0 left-0 z-10 flex items-center justify-between px-5 py-2 text-sm'>
        <Link to='/' className='pointer-events-auto hover:text-sky-600'>
          ← 시뮬레이터
        </Link>
        <span className='pointer-events-auto flex items-center gap-2'>
          <button
            onClick={() => go(-1)}
            className='border-border bg-card rounded-md border px-2 py-0.5 hover:border-sky-500'
          >
            ◀
          </button>
          <span>
            {cur + 1} / {n}
          </span>
          <button
            onClick={() => go(1)}
            className='border-border bg-card rounded-md border px-2 py-0.5 hover:border-sky-500'
          >
            ▶
          </button>
        </span>
      </div>
    </div>
  );
}
