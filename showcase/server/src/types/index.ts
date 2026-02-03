/** Server types: parts, API status. */
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
