/** Global state: highlights, selection, parts, camera, sound settings. */
import { create } from 'zustand';
import type { RobotState, RobotActions, RobotPart, ApiStatus } from '../types/robot';

export const useRobotStore = create<RobotState & RobotActions>((set) => ({
  highlightedParts: [],
  selectedPart: null,
  isLoading: false,
  error: null,
  parts: [],
  apiStatus: null,
  explodeStrength: 25,
  showGround: false,
  groundY: -0.15,
  cameraMode: 'third',
  tourActive: false,

  // Sound settings (for UI sounds)
  soundEnabled: true,
  soundVolume: 0.5,

  dronePosition: { x: 0, y: 0, z: 0 },
  droneOffset: null,
  droneSpeed: 0,
  isDroneMoving: false,

  highlightParts: (partIds: string[]) => set({ highlightedParts: partIds, error: null }),
  clearHighlights: () => set({ highlightedParts: [], selectedPart: null }),
  selectPart: (partId: string | null) => set({ selectedPart: partId }),
  setLoading: (loading: boolean) => set({ isLoading: loading }),
  setError: (error: string | null) => set({ error }),
  setParts: (parts: RobotPart[]) => set({ parts }),
  setApiStatus: (status: ApiStatus) => set({ apiStatus: status }),
  setExplodeStrength: (strength: number) => set({ explodeStrength: strength }),
  toggleGround: () => set((state) => ({ showGround: !state.showGround })),
  setGroundY: (y: number) => set({ groundY: y }),
  toggleCameraMode: () =>
    set((state) => ({ cameraMode: state.cameraMode === 'third' ? 'first' : 'third' })),
  setTourActive: (active: boolean) => set({ tourActive: active }),

  setDroneOffset: (offset) => set({ droneOffset: offset }),
  setDronePosition: (pos) =>
    set((state) => ({
      dronePosition: typeof pos === 'function' ? pos(state.dronePosition) : pos,
    })),
  setDroneSpeed: (speed) => set({ droneSpeed: speed }),
  setDroneMoving: (moving) => set({ isDroneMoving: moving }),

  // Sound settings actions
  setSoundEnabled: (enabled: boolean) => set({ soundEnabled: enabled }),
  setSoundVolume: (volume: number) => set({ soundVolume: volume }),
}));
