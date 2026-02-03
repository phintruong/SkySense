/** UI sound metadata. Migrated from server. */

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

export function getAvailableUISounds(): UISound[] {
  return UI_SOUNDS;
}

export function getUISoundById(id: string): UISound | undefined {
  return UI_SOUNDS.find((sound) => sound.id === id);
}
