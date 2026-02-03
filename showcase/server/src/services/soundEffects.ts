/** Engine + UI sounds. List, serve. Generation removed (no external API). */

export interface EngineSound {
  id: string;
  name: string;
  description: string;
  prompt: string;
}

export interface UISound {
  id: string;
  name: string;
  filename: string;
}

export const UI_SOUNDS: UISound[] = [
  {
    id: 'select-part',
    name: 'Select Part',
    filename: 'select-part.mp3',
  },
  {
    id: 'hover-part',
    name: 'Hover Part',
    filename: 'hover-part.mp3',
  },
  {
    id: 'attach-part',
    name: 'Attach Part',
    filename: 'attach-part.mp3',
  },
  {
    id: 'detach-part',
    name: 'Detach Part',
    filename: 'detach-part.mp3',
  },
  {
    id: 'whoosh',
    name: 'Whoosh',
    filename: 'whoosh.mp3',
  },
  {
    id: 'success',
    name: 'Success',
    filename: 'success.mp3',
  },
  {
    id: 'error',
    name: 'Error',
    filename: 'error.mp3',
  },
];

export const ENGINE_SOUNDS: EngineSound[] = [
  {
    id: 'sports-car',
    name: 'Sports Car',
    description: 'High-performance sports car engine',
    prompt: 'sports car engine revving, high RPM, powerful V8 engine sound',
  },
  {
    id: 'electric-motor',
    name: 'Electric Motor',
    description: 'Quiet electric vehicle motor',
    prompt: 'electric car motor whirring, futuristic EV sound, quiet hum',
  },
  {
    id: 'muscle-car',
    name: 'Muscle Car',
    description: 'Classic American muscle car',
    prompt: 'classic muscle car engine rumbling, deep V8 growl, American muscle',
  },
  {
    id: 'formula-one',
    name: 'Formula One',
    description: 'High-pitched F1 race car',
    prompt: 'Formula 1 racing car engine, high pitched whine, screaming race engine',
  },
  {
    id: 'diesel-truck',
    name: 'Diesel Truck',
    description: 'Heavy diesel truck engine',
    prompt: 'diesel truck engine rumbling, heavy machinery, deep diesel sound',
  },
  {
    id: 'motorcycle',
    name: 'Motorcycle',
    description: 'Powerful motorcycle engine',
    prompt: 'motorcycle engine revving, Harley Davidson rumble, powerful bike',
  },
  {
    id: 'go-kart',
    name: 'Go Kart',
    description: 'Small go-kart engine',
    prompt: 'go kart engine buzzing, small racing engine, high pitched kart sound',
  },
  {
    id: 'spaceship',
    name: 'Spaceship',
    description: 'Futuristic spaceship thruster',
    prompt: 'spaceship engine humming, sci-fi thruster sound, futuristic propulsion',
  },
];

export async function generateSoundEffect(_prompt: string, _duration: number = 2): Promise<Buffer | null> {
  return null;
}

export function getAvailableSounds(): EngineSound[] {
  return ENGINE_SOUNDS;
}

export function getSoundById(id: string): EngineSound | undefined {
  return ENGINE_SOUNDS.find((sound) => sound.id === id);
}

export function getAvailableUISounds(): UISound[] {
  return UI_SOUNDS;
}

export function getUISoundById(id: string): UISound | undefined {
  return UI_SOUNDS.find((sound) => sound.id === id);
}
