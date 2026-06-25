import { useCallback, useEffect, useRef, useState } from 'react';

// 백엔드 /ws/interactive 스트림 한 프레임
export interface InteractiveFrame {
  theta: number[];
  ee_pos: number[];
  flags: string[];
  diverged: boolean;
}

export interface InteractiveConfig {
  robot_id: string;
  controller: string;
  gains: Record<string, number>;
  mode: 'honest' | 'safe';
}

/**
 * 실시간 grab & push WebSocket 훅.
 * - active=true 면 /ws/interactive 연결 → config 전송 → θ 스트림을 onFrame 으로 전달.
 * - config(제어기/게인/모드) 변경 시 재전송(세션 재설정).
 * - sendGrab(force[3]) / reset() 으로 잡기힘·리셋 전송.
 * 물리는 백엔드가 1kHz 로 돌리고, 여기선 프레임만 받아 렌더한다(프레임당 REST 금지 원칙은
 * REST 폴링 얘기 — WS 스트리밍은 OK).
 */
export function useInteractive(
  active: boolean,
  config: InteractiveConfig,
  onFrame: (f: InteractiveFrame) => void,
) {
  const wsRef = useRef<WebSocket | null>(null);
  const [connected, setConnected] = useState(false);
  const onFrameRef = useRef(onFrame);
  onFrameRef.current = onFrame;
  const cfgRef = useRef(config);
  cfgRef.current = config;

  // 연결: active 토글에만 반응(게인 바뀐다고 재연결하지 않도록)
  useEffect(() => {
    if (!active) return;
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    const ws = new WebSocket(`${proto}://${location.host}/ws/interactive`);
    wsRef.current = ws;
    ws.onopen = () => {
      setConnected(true);
      ws.send(JSON.stringify({ type: 'config', ...cfgRef.current }));
    };
    ws.onmessage = (e) => {
      const f = JSON.parse(e.data);
      if (!f.error) onFrameRef.current(f as InteractiveFrame);
    };
    ws.onclose = () => setConnected(false);
    return () => {
      ws.close();
      wsRef.current = null;
      setConnected(false);
    };
  }, [active]);

  // config 변경 → 세션 재설정(연결 유지된 채)
  const cfgKey = `${config.robot_id}|${config.controller}|${config.mode}|${JSON.stringify(config.gains)}`;
  useEffect(() => {
    const ws = wsRef.current;
    if (active && ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ type: 'config', ...cfgRef.current }));
    }
  }, [active, cfgKey]);

  const send = (msg: object) => {
    const ws = wsRef.current;
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg));
  };
  const sendGrab = useCallback((force: number[]) => {
    send({ type: 'grab', force });
  }, []);
  const reset = useCallback(() => {
    send({ type: 'reset' });
  }, []);

  return { connected, sendGrab, reset };
}
