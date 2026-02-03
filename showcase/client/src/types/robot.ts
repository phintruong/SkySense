/** Types for parts and state. */
export type PartCategory = 'motion' | 'sensors' | 'power' | 'control' | 'structure' | 'communication';

export interface RobotPart {
  id: string;
  name: string;
  description: string;
  keywords: string[];
  category: PartCategory;
  relatedTo?: string[];
  functionalRole?: string;
}

export interface ApiStatus {
  demoMode: boolean;
}

export type Vector3Like = { x: number; y: number; z: number };

export interface RobotState {
  highlightedParts: string[];
  selectedPart: string | null;
  isLoading: boolean;
  error: string | null;
  parts: RobotPart[];
  apiStatus: ApiStatus | null;
  explodeStrength: number;
  showGround: boolean;
  groundY: number;
  cameraMode: 'third' | 'first';
  tourActive: boolean;
  soundEnabled: boolean;
  soundVolume: number;
  /** Drone position when driving (offset from initial center). */
  dronePosition: Vector3Like;
  /** Initial drone center in world space (set when model loads). */
  droneOffset: Vector3Like | null;
  /** Current horizontal speed (for UI/sound). */
  droneSpeed: number;
  /** True when drone is moving (for UI/sound). */
  isDroneMoving: boolean;
  /** True when the parts panel (part mover) is open. */
  partsPanelOpen: boolean;
}

export interface RobotActions {
  highlightParts: (partIds: string[]) => void;
  clearHighlights: () => void;
  selectPart: (partId: string | null) => void;
  setLoading: (loading: boolean) => void;
  setError: (error: string | null) => void;
  setParts: (parts: RobotPart[]) => void;
  setApiStatus: (status: ApiStatus) => void;
  setExplodeStrength: (strength: number) => void;
  toggleGround: () => void;
  setGroundY: (y: number) => void;
  toggleCameraMode: () => void;
  setTourActive: (active: boolean) => void;
  setSoundEnabled: (enabled: boolean) => void;
  setSoundVolume: (volume: number) => void;
  setDroneOffset: (offset: Vector3Like | null) => void;
  setDronePosition: (pos: Vector3Like | ((prev: Vector3Like) => Vector3Like)) => void;
  setDroneSpeed: (speed: number) => void;
  setDroneMoving: (moving: boolean) => void;
  setPartsPanelOpen: (open: boolean) => void;
}
