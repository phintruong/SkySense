import { useGNCStore } from "../hooks/useGNCStore";

export function ConnectionBadge() {
  const connected = useGNCStore((state) => state.connected);

  return (
    <div className="absolute right-4 top-4 z-10 flex items-center gap-2 rounded border border-gray-800 bg-gray-950/85 px-3 py-2 text-xs font-semibold tracking-wide text-gray-200 shadow-lg">
      <span
        className={`h-2 w-2 rounded-full ${
          connected ? "bg-emerald-400" : "animate-pulse bg-red-400"
        }`}
      />
      {connected ? "LIVE" : "DISCONNECTED"}
    </div>
  );
}
