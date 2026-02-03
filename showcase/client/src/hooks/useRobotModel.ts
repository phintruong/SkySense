/** Global state: highlights, selection, parts, camera, sound. */
import { create } from 'zustand';
import type { RobotState, RobotActions, RobotPart, ApiStatus, EngineSound } from '../types/robot';

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

  // Sound state
  soundEnabled: true,
  soundVolume: 0.5,
  selectedSound: null,
  availableSounds: [],
  isLoadingSound: false,
  carSpeed: 0,
  isCarMoving: false,

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

  // Sound actions
  setSoundEnabled: (enabled: boolean) => set({ soundEnabled: enabled }),
  setSoundVolume: (volume: number) => set({ soundVolume: volume }),
  setSelectedSound: (sound: EngineSound | null) => set({ selectedSound: sound }),
  setAvailableSounds: (sounds: EngineSound[]) => set({ availableSounds: sounds }),
  setIsLoadingSound: (loading: boolean) => set({ isLoadingSound: loading }),
  setCarSpeed: (speed: number) => set({ carSpeed: speed }),
  setIsCarMoving: (moving: boolean) => set({ isCarMoving: moving }),
}));
