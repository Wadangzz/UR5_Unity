import { useCallback, useEffect, useState, type ReactNode } from 'react';
import { Link } from 'react-router-dom';
import katex from 'katex';
import { Card, CardContent } from '@/components/ui/card';
import { Button } from '@/components/ui/button';

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
const Cols = ({ l, r }: { l: ReactNode; r: ReactNode }) => (
  <div className='flex items-center gap-10'>
    <div className='flex-[1.1]'>{l}</div>
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

/* ── SVG 도식 (raw) ── */
const SCREW = `<svg viewBox="0 0 190 185" width="210"><line x1="95" y1="172" x2="95" y2="20" stroke="#64748b" stroke-width="2" stroke-dasharray="4 4"/><text x="102" y="28" fill="#64748b" font-size="12">축 S</text><path d="M65,148 C65,134 125,134 125,118 C125,102 65,102 65,86 C65,70 125,70 125,54 C125,40 65,40 65,28" fill="none" stroke="#0284c7" stroke-width="3.5"/><line x1="95" y1="166" x2="95" y2="138" stroke="#059669" stroke-width="3" marker-end="url(#ag)"/><text x="100" y="158" fill="#059669" font-size="12">직진</text><path d="M123,116 A29,10 0 0,1 87,116" fill="none" stroke="#d97706" stroke-width="2" marker-end="url(#aw)"/><text x="127" y="110" fill="#d97706" font-size="12">회전</text></svg>`;
const MANIP = `<svg viewBox="0 0 340 180" width="330"><ellipse cx="138" cy="98" rx="28" ry="18" transform="rotate(20 138 98)" fill="rgba(52,211,153,.18)" stroke="#059669" stroke-width="2"/><line x1="35" y1="140" x2="78" y2="72" stroke="#64748b" stroke-width="6"/><line x1="78" y1="72" x2="138" y2="98" stroke="#64748b" stroke-width="6"/><rect x="29" y="137" width="12" height="8" fill="#27344d"/><circle cx="78" cy="72" r="4.5" fill="#3b82f6"/><circle cx="138" cy="98" r="4.5" fill="#d97706"/><text x="95" y="166" fill="#059669" font-size="12.5" text-anchor="middle">정상 — 둥근</text><ellipse cx="291" cy="28" rx="7" ry="25" transform="rotate(-53 291 28)" fill="rgba(251,191,36,.18)" stroke="#d97706" stroke-width="2"/><line x1="205" y1="140" x2="248" y2="84" stroke="#64748b" stroke-width="6"/><line x1="248" y1="84" x2="291" y2="28" stroke="#64748b" stroke-width="6"/><rect x="199" y="137" width="12" height="8" fill="#27344d"/><circle cx="248" cy="84" r="4.5" fill="#3b82f6"/><circle cx="291" cy="28" r="4.5" fill="#d97706"/><text x="252" y="166" fill="#d97706" font-size="12.5" text-anchor="middle">특이점 — 납작</text></svg>`;
const DAMP = `<svg viewBox="0 0 240 150" width="290"><line x1="30" y1="115" x2="225" y2="115" stroke="#27344d"/><line x1="30" y1="25" x2="30" y2="115" stroke="#27344d"/><line x1="30" y1="52" x2="225" y2="52" stroke="#64748b" stroke-dasharray="4 4"/><text x="200" y="47" fill="#64748b" font-size="11">목표</text><path d="M30,115 C70,8 95,72 120,55 C140,45 170,53 225,52" fill="none" stroke="#d97706" stroke-width="2.5"/><path d="M30,115 C80,52 130,52 225,52" fill="none" stroke="#059669" stroke-width="2.5"/><path d="M30,115 C110,92 170,57 225,53" fill="none" stroke="#0284c7" stroke-width="2.5"/><text x="118" y="18" fill="#d97706" font-size="11">ζ&lt;1</text><text x="150" y="42" fill="#059669" font-size="11">ζ=1</text><text x="162" y="78" fill="#0284c7" font-size="11">ζ&gt;1</text></svg>`;
const MSD = `<svg viewBox="0 0 200 165" width="210"><line x1="20" y1="28" x2="180" y2="28" stroke="#64748b" stroke-width="3"/><path d="M100,28 l-10,12 l20,12 l-20,12 l20,12 l-10,12" fill="none" stroke="#0284c7" stroke-width="2.5"/><line x1="130" y1="28" x2="130" y2="76" stroke="#059669" stroke-width="2.5"/><rect x="124" y="58" width="12" height="18" fill="none" stroke="#059669" stroke-width="2"/><rect x="78" y="92" width="48" height="32" rx="5" fill="rgba(59,130,246,.2)" stroke="#3b82f6" stroke-width="2"/><text x="102" y="113" fill="#0f172a" font-size="13" text-anchor="middle">m</text><text x="60" y="58" fill="#0284c7" font-size="13">K</text><text x="142" y="56" fill="#059669" font-size="13">B</text><line x1="102" y1="124" x2="102" y2="150" stroke="#d97706" stroke-width="2.5" marker-end="url(#aw)"/></svg>`;
const CHALK = `<svg viewBox="0 0 220 175" width="240"><rect x="150" y="20" width="22" height="145" fill="rgba(56,189,248,.12)" stroke="#0284c7" stroke-width="2"/><text x="161" y="14" fill="#0284c7" font-size="12" text-anchor="middle">벽</text><line x1="60" y1="92" x2="148" y2="92" stroke="#64748b" stroke-width="6"/><circle cx="148" cy="92" r="6" fill="#d97706"/><line x1="120" y1="92" x2="148" y2="92" stroke="#059669" stroke-width="3" marker-end="url(#ag)"/><text x="116" y="83" fill="#059669" font-size="12">법선 힘</text><line x1="148" y1="118" x2="148" y2="52" stroke="#3b82f6" stroke-width="2.5" stroke-dasharray="5 4" marker-end="url(#a)"/><text x="153" y="138" fill="#3b82f6" font-size="12">접선 이동</text></svg>`;
const GRAV = `<svg viewBox="0 0 190 150" width="210"><line x1="40" y1="50" x2="40" y2="140" stroke="#27344d" stroke-width="6"/><line x1="40" y1="50" x2="145" y2="50" stroke="#64748b" stroke-width="7"/><circle cx="40" cy="50" r="7" fill="#3b82f6"/><line x1="92" y1="58" x2="92" y2="100" stroke="#d97706" stroke-width="2.5" marker-end="url(#aw)"/><line x1="145" y1="58" x2="145" y2="100" stroke="#d97706" stroke-width="2.5" marker-end="url(#aw)"/><text x="112" y="118" fill="#d97706" font-size="12">중력</text><path d="M30,65 A18,18 0 0,1 28,42" fill="none" stroke="#059669" stroke-width="2" marker-end="url(#ag)"/><text x="6" y="48" fill="#059669" font-size="13">τ</text></svg>`;

type S = {
  kicker?: string;
  title?: ReactNode;
  body: ReactNode;
  divider?: boolean;
};

const SLIDES: S[] = [
  {
    body: (
      <div className='text-center'>
        <div className='text-muted-foreground text-2xl'>Modern Robotics</div>
        <h1 className='my-3 text-5xl leading-tight font-extrabold'>
          로봇 기구학 · 동역학 · 제어
        </h1>
      </div>
    ),
  },
  {
    title: '로봇을 똑똑하게 움직이려면? — 세 가지',
    body: (
      <>
        <UL
          items={[
            <>
              ① <b>어디로</b> — 기구학 (관절각 ↔ 손끝 위치)
            </>,
            <>
              ② <b>어떤 힘으로</b> — 동역학 (힘 ↔ 운동)
            </>,
            <>
              ③ <b>원하는 대로</b> — 제어 (목표 수렴 · 환경 상호작용)
            </>,
          ]}
        />
        <Eq t='\text{공통 언어: 회전+이동을 ‘나사 하나(Screw)’로 — 좌표 무관·기하적}' />
      </>
    ),
  },
  {
    title: "핵심 아이디어 — 회전+이동을 '나사 하나'로",
    body: (
      <Cols
        l={
          <>
            <UL
              items={[
                <>
                  오일러각은 <span className='text-amber-600'>짐벌락</span> 함정
                </>,
                <>
                  <b>Chasles:</b> 모든 강체운동 = 한 축으로 회전+직진 = 나사
                </>,
                <>
                  회전·직동 관절을 <A>한 스크류축으로 통일</A>
                </>,
              ]}
            />
            <Ex>
              <b>비유:</b> 병뚜껑·드릴 — 돌면서 들어간다.
            </Ex>
          </>
        }
        r={<Fig svg={SCREW} cap='Screw = 회전+직진' />}
      />
    ),
  },
  {
    title: '기구학 — FK / IK / 특이점',
    body: (
      <Cols
        l={
          <UL
            items={[
              <>
                <b>FK:</b> 관절각 → 손끝 위치
              </>,
              <>
                <b>IK:</b> 손끝 위치 → 관절각 (반복)
              </>,
              <>
                <b>특이점:</b> ‘팔 펴면 못 움직임’ → 속도 폭주
              </>,
              <>
                → <A>조작성 타원</A> 납작, DLS로 회피
              </>,
            ]}
          />
        }
        r={<Fig svg={MANIP} cap='조작성 타원' />}
      />
    ),
  },
  {
    title: '동역학 — 힘과 운동',
    body: (
      <Cols
        l={
          <>
            <Eq t='M(\theta)\ddot\theta + c(\theta,\dot\theta) + g(\theta) = \tau' />
            <UL
              items={[
                <>
                  관성 <Tex t='M' /> · 코리올리 <Tex t='c' /> · 중력{' '}
                  <Tex t='g' />
                </>,
                <>
                  중력 <Tex t='g(\theta)' />: 팔 뻗으면 어깨에 큰 토크
                </>,
              ]}
            />
            <Num items={['Newton-Euler (RNE)', 'Numba JIT ~14×']} />
          </>
        }
        r={<Fig svg={GRAV} cap='중력 토크 보상' />}
      />
    ),
  },
  {
    title: '제어의 직관 — 스프링처럼 (2차계)',
    body: (
      <Cols
        l={
          <>
            <UL
              items={[
                <>
                  목표로 당기는 <b>스프링(Kp)</b> + 흔들림 잡는 <b>댐퍼(Kd)</b>
                </>,
                <>
                  감쇠비 <Tex t='\zeta' />: 출렁(&lt;1) / 딱(=1) / 느림(&gt;1)
                </>,
              ]}
            />
            <Eq t='\ddot e + 2\zeta\omega_n\dot e + \omega_n^2 e = 0' />
            <Ex>
              <b>예:</b> 자동차 서스펜션 균형이 <Tex t='\zeta\approx1' />.
            </Ex>
          </>
        }
        r={<Fig svg={DAMP} cap='감쇠비별 응답' />}
      />
    ),
  },
  {
    title: '관절공간 제어',
    body: (
      <>
        <UL
          items={[
            <>
              <b>PD+중력보상</b> <Tex t='\tau=K_p e+K_d\dot e+\tilde g' /> —
              목표에 정확히
            </>,
            <>
              <b>PID</b> — 적분(I)이 마찰·오차 잔류오차 제거
            </>,
            <>
              <b>Computed Torque</b> — 비선형 상쇄 → 선형계
            </>,
          ]}
        />
        <Eq t='\tau=M(\theta)\big(\ddot\theta_d+K_p e+K_d\dot e+K_i\!\int e\big)+c+g' />
      </>
    ),
  },
  {
    title: '작업공간(직교) 제어',
    body: (
      <>
        <UL
          items={[
            <>
              말단 위치/자세를 직접 — <b>Resolved-Rate</b>
            </>,
          ]}
        />
        <Eq t='\mathcal{V}_b=[\mathrm{Ad}]\mathcal{V}_d+K_pX_e,\quad \dot\theta=J_b^{\dagger}\mathcal{V}_b' />
        <UL
          items={[
            <>
              <A>궤적 ⊥ 제어기</A> — 직선 추종 이탈 0.0mm
            </>,
          ]}
        />
      </>
    ),
  },
  {
    title: '임피던스 ↔ 어드미턴스',
    body: (
      <Cols
        l={
          <>
            <UL
              items={[
                <>
                  <b>임피던스</b>: 움직임 보고 힘 (가상 스프링)
                </>,
                <>
                  <b>어드미턴스</b>: 힘 느끼고 움직임
                </>,
                <>
                  역수 <Tex t='Z=F/X \leftrightarrow Y=X/F' />
                </>,
              ]}
            />
            <Ex>
              <b>예:</b> 악수=임피던스, 무거운 문=어드미턴스.
            </Ex>
          </>
        }
        r={<Fig svg={MSD} cap='가상 질량-스프링-댐퍼' />}
      />
    ),
  },
  {
    title: '힘 & 하이브리드 제어',
    body: (
      <Cols
        l={
          <>
            <UL
              items={[
                <>
                  <b>힘 제어:</b> 벽을 목표 힘으로 누름
                </>,
                <>
                  <b>하이브리드:</b>{' '}
                  <span className='text-emerald-600'>법선=힘</span> ·{' '}
                  <A>접선=위치</A> 동시
                </>,
              ]}
            />
            <Num items={['힘 20N · 오차 0%', '접선오차 0.8mm']} />
          </>
        }
        r={<Fig svg={CHALK} cap='누르며 선 긋기' />}
      />
    ),
  },
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
            ['Resolved-rate', 'moveit_servo (Twist)', 'RmpFlow · Jacobian'],
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
            '회전=Lie 군 · 운동=Screw',
            '한 틀로 기구학·동역학·제어',
            'PD·PID·CT·임피던스·힘·하이브리드',
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
    title: 'Body Jacobian & Singularity',
    body: (
      <Cols
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
    title: 'PoE 정기구학 & 수치 IK',
    body: (
      <>
        <Eq t='T(\theta)=e^{[\mathcal{S}_1]\theta_1}\cdots e^{[\mathcal{S}_n]\theta_n}\,M,\quad \mathcal{S}_i=(\omega_i,\,-\omega_i\times q_i)' />
        <Eq t='\mathcal{V}_b=\log(T^{-1}T_d),\quad \theta\leftarrow\theta+J_b^{\dagger}\mathcal{V}_b' />
        <Ex>
          <b>DH 대비:</b> 단일 프레임 · 회전·직동 통일 · URDF서 바로 추출.
        </Ex>
      </>
    ),
  },
  {
    title: '동역학 — Newton-Euler (RNE) & 제어 법칙',
    body: (
      <>
        <Eq t='M(\theta)=\sum_i J_i^\top \mathcal{G}_i J_i,\quad \ddot\theta=M^{-1}(\tau-c-g-J^\top\mathcal{F}_{tip})' />
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
