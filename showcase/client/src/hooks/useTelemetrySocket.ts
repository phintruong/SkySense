import { useEffect } from "react";
import type { Command, TelemetryMessage } from "../types/telemetry";
import { useGNCStore } from "./useGNCStore";

const WS_URL = "ws://localhost:8000/ws/telemetry";
const RECONNECT_DELAY = 2000;

export function useTelemetrySocket() {
  const setConnected = useGNCStore((state) => state.setConnected);
  const updateTelemetry = useGNCStore((state) => state.updateTelemetry);
  const setSendCommand = useGNCStore((state) => state.setSendCommand);

  useEffect(() => {
    let ws: WebSocket | null = null;
    let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
    let shouldReconnect = true;

    const connect = () => {
      if (
        ws?.readyState === WebSocket.OPEN ||
        ws?.readyState === WebSocket.CONNECTING
      ) {
        return;
      }

      ws = new WebSocket(WS_URL);

      ws.onopen = () => {
        setConnected(true);
        setSendCommand((cmd: Command) => {
          if (ws?.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify(cmd));
          }
        });
      };

      ws.onmessage = (event) => {
        try {
          const msg: TelemetryMessage = JSON.parse(event.data);
          if (msg.type === "telemetry") {
            updateTelemetry(msg);
          }
        } catch {
          // Ignore malformed frames and wait for the next telemetry message.
        }
      };

      ws.onclose = () => {
        setConnected(false);
        setSendCommand(null);
        ws = null;
        if (shouldReconnect) {
          reconnectTimer = setTimeout(connect, RECONNECT_DELAY);
        }
      };

      ws.onerror = () => {
        ws?.close();
      };
    };

    connect();

    return () => {
      shouldReconnect = false;
      if (reconnectTimer) {
        clearTimeout(reconnectTimer);
      }
      ws?.close();
    };
  }, [setConnected, setSendCommand, updateTelemetry]);
}
