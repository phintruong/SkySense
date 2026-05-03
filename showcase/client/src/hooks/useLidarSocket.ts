/** WebSocket hook: connects to the Python backend telemetry stream. */
import { useEffect, useRef } from 'react';
import { useRobotStore } from './useRobotModel';

const WS_URL = 'ws://localhost:8000/ws/telemetry';
const RECONNECT_DELAY = 2000;

interface TelemetryMessage {
  type: 'telemetry';
  true_state?: {
    position?: number[];
    velocity?: number[];
  };
}

export function useTelemetrySocket() {
  const setLidarConnected = useRobotStore((s) => s.setLidarConnected);
  const setLidarData = useRobotStore((s) => s.setLidarData);
  const setDronePosition = useRobotStore((s) => s.setDronePosition);
  const setDroneSpeed = useRobotStore((s) => s.setDroneSpeed);
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimer = useRef<ReturnType<typeof setTimeout> | null>(null);

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
          const msg = JSON.parse(event.data) as TelemetryMessage;
          if (msg.type !== 'telemetry') return;

          const position = msg.true_state?.position;
          if (Array.isArray(position) && position.length === 3) {
            setDronePosition({ x: position[1], y: -position[2], z: position[0] });
          }

          const velocity = msg.true_state?.velocity;
          if (Array.isArray(velocity) && velocity.length === 3) {
            setDroneSpeed(Math.hypot(velocity[0], velocity[1], velocity[2]));
          }

          setLidarData({ scan: [], action: '', obstacles: [], tiltAngle: 0 });
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
      if (reconnectTimer.current !== null) {
        clearTimeout(reconnectTimer.current);
      }
      wsRef.current?.close();
    };
  }, [setDronePosition, setDroneSpeed, setLidarConnected, setLidarData]);
}

export const useLidarSocket = useTelemetrySocket;
