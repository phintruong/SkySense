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
}
