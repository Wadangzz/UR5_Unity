import {
  CartesianGrid,
  Line,
  LineChart,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from 'recharts';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import type { RunResponse } from '@/api';

const JOINT_COLORS = [
  '#ef4444',
  '#f59e0b',
  '#10b981',
  '#3b82f6',
  '#8b5cf6',
  '#ec4899',
];

/** 마지막 Run 결과의 추종오차·관절토크 시계열 그래프. */
export default function Plots({ result }: { result: RunResponse | null }) {
  if (!result) return null;

  const data = result.t.map((t, i) => {
    const row: Record<string, number> = {
      t: Number(t.toFixed(2)),
      err: result.error[i],
    };
    result.torque[i]?.forEach((tau, j) => {
      row[`j${j}`] = tau;
    });
    return row;
  });

  const axis = { fontSize: 10, stroke: '#94a3b8' };

  return (
    <Card className='bg-card/95 supports-[backdrop-filter]:bg-card/80 w-[560px] backdrop-blur'>
      <CardHeader className='pb-2'>
        <CardTitle className='text-sm'>시뮬 결과</CardTitle>
      </CardHeader>
      <CardContent className='flex gap-4'>
        <div className='flex-1'>
          <p className='text-muted-foreground mb-1 text-xs'>
            추종오차 ‖θ − θ_d‖ (rad)
          </p>
          <ResponsiveContainer width='100%' height={130}>
            <LineChart
              data={data}
              margin={{ top: 4, right: 8, bottom: 0, left: -8 }}
            >
              <CartesianGrid strokeDasharray='3 3' stroke='#e2e8f0' />
              <XAxis dataKey='t' tick={axis} unit='s' />
              <YAxis tick={axis} width={38} />
              <Tooltip contentStyle={{ fontSize: 11 }} />
              <Line
                type='monotone'
                dataKey='err'
                stroke='#0f172a'
                dot={false}
                strokeWidth={1.5}
              />
            </LineChart>
          </ResponsiveContainer>
        </div>

        <div className='flex-1'>
          <p className='text-muted-foreground mb-1 text-xs'>
            관절 토크 τ (N·m)
          </p>
          <ResponsiveContainer width='100%' height={130}>
            <LineChart
              data={data}
              margin={{ top: 4, right: 8, bottom: 0, left: -8 }}
            >
              <CartesianGrid strokeDasharray='3 3' stroke='#e2e8f0' />
              <XAxis dataKey='t' tick={axis} unit='s' />
              <YAxis tick={axis} width={38} />
              <Tooltip contentStyle={{ fontSize: 11 }} />
              {JOINT_COLORS.map((c, j) => (
                <Line
                  key={j}
                  type='monotone'
                  dataKey={`j${j}`}
                  stroke={c}
                  dot={false}
                  strokeWidth={1.2}
                />
              ))}
            </LineChart>
          </ResponsiveContainer>
        </div>
      </CardContent>
    </Card>
  );
}
