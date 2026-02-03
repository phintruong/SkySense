/**
 * Maps robot component IDs to their corresponding sound effect IDs
 */

export type ComponentSoundId =
  | 'wheel'
  | 'sensor'
  | 'ultrasonic-sensor'
  | 'color-sensor'
  | 'arduino'
  | 'motor-driver'
  | 'battery'
  | 'breadboard'
  | 'plate';

/**
 * Maps component IDs from the robot model to their sound effect IDs
 */
export function getComponentSound(componentId: string): ComponentSoundId {
  // Handle numbered variants (e.g., wheel-1, wheel-2 -> wheel)
  const baseId = componentId.replace(/-\d+$/, '');

  // Direct matches
  const soundMap: Record<string, ComponentSoundId> = {
    'wheel': 'wheel',
    'sensor': 'sensor',
    'ultrasonic-sensor': 'ultrasonic-sensor',
    'color-sensor': 'color-sensor',
    'arduino': 'arduino',
    'motor-driver': 'motor-driver',
    'battery': 'battery',
    'breadboard': 'breadboard',
    'plate': 'plate',
  };

  // Return mapped sound or default to sensor for unknown components
  return soundMap[baseId] || 'sensor';
}

/**
 * Get the file path for a component sound
 */
export function getComponentSoundPath(componentId: string): string {
  const soundId = getComponentSound(componentId);
  return `/sounds/components/${soundId}.mp3`;
}
