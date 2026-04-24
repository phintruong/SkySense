/** WebSocket hook: connects to the Python backend and streams LiDAR data into Zustand. */
import { useEffect, useRef } from 'react';
import { useRobotStore } from './useRobotModel';

const WS_URL = 'ws://localhost:8000/ws/lidar';
const RECONNECT_DELAY = 2000;

export function useLidarSocket() {
  const setLidarConnected = useRobotStore((s) => s.setLidarConnected);
  const setLidarData = useRobotStore((s) => s.setLidarData);
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimer = useRef<ReturnType<typeof setTimeout>>();

  useEffect(() => {
    let unmounted = false;

    function connect() {
      if (unmounted) return;
      const ws = new WebSocket(WS_URL);
      wsRef.current = ws;

      ws.onopen = () => {
        if (!unmounted) setLidarConnected(true);
      };

      ws.onmessage = (event) => {
        try {
          const msg = JSON.parse(event.data);
          const scan = (msg.scan as [number, number][]).map(([angle, distance]) => ({ angle, distance }));
          setLidarData({
            scan,
            action: msg.action,
            obstacles: msg.obstacles,
            tiltAngle: msg.tiltAngle,
          });
        } catch {
          // ignore malformed messages
        }
      };

      ws.onclose = () => {
        if (!unmounted) {
          setLidarConnected(false);
          reconnectTimer.current = setTimeout(connect, RECONNECT_DELAY);
        }
      };

      ws.onerror = () => {
        ws.close();
      };
    }

    connect();

    return () => {
      unmounted = true;
      clearTimeout(reconnectTimer.current);
      wsRef.current?.close();
    };
  }, [setLidarConnected, setLidarData]);
}
