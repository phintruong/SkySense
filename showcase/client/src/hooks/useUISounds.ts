/** Plays UI sounds (click, hover, success) on part interaction. */
import { useEffect, useRef, useCallback } from 'react';
import { useRobotStore } from './useRobotModel';
import { type ComponentSoundId, getComponentSound } from '../utils/componentSounds';

type UISoundId = 'select-part' | 'hover-part' | 'attach-part' | 'detach-part' | 'whoosh' | 'success' | 'error' | 'drift';

interface UISoundMap {
  [key: string]: HTMLAudioElement;
}

export function useUISounds() {
  const soundsRef = useRef<UISoundMap>({});
  const componentSoundsRef = useRef<UISoundMap>({});
  const soundEnabled = useRobotStore((s) => s.soundEnabled);
  const soundVolume = useRobotStore((s) => s.soundVolume);

  // Preload all UI sounds
  useEffect(() => {
    const soundIds: UISoundId[] = [
      'select-part',
      'hover-part',
      'attach-part',
      'detach-part',
      'whoosh',
      'success',
      'error',
      'drift',
    ];

    soundIds.forEach((id) => {
      const audio = new Audio(`/sounds/ui/${id}.mp3`);
      audio.preload = 'auto';
      audio.volume = soundVolume * 0.5; // UI sounds at 50% of global volume
      soundsRef.current[id] = audio;
    });

    // Cleanup
    return () => {
      Object.values(soundsRef.current).forEach((audio) => {
        audio.pause();
        audio.src = '';
      });
      soundsRef.current = {};
    };
  }, []);

  // Preload all component sounds
  useEffect(() => {
    const componentSoundIds: ComponentSoundId[] = [
      'wheel',
      'sensor',
      'ultrasonic-sensor',
      'color-sensor',
      'arduino',
      'motor-driver',
      'battery',
      'breadboard',
      'plate',
    ];

    componentSoundIds.forEach((id) => {
      const audio = new Audio(`/sounds/components/${id}.mp3`);
      audio.preload = 'auto';
      audio.volume = soundVolume * 0.6; // Component sounds at 60% of global volume
      componentSoundsRef.current[id] = audio;
    });

    // Cleanup
    return () => {
      Object.values(componentSoundsRef.current).forEach((audio) => {
        audio.pause();
        audio.src = '';
      });
      componentSoundsRef.current = {};
    };
  }, []);

  // Update volume when global volume changes
  useEffect(() => {
    Object.values(soundsRef.current).forEach((audio) => {
      audio.volume = soundVolume * 0.5;
    });
    Object.values(componentSoundsRef.current).forEach((audio) => {
      audio.volume = soundVolume * 0.6;
    });
  }, [soundVolume]);

  const playSound = useCallback(
    (soundId: UISoundId, volumeMultiplier: number = 1.0) => {
      if (!soundEnabled) return;

      const audio = soundsRef.current[soundId];
      if (!audio) return;

      // Reset and play
      audio.currentTime = 0;
      audio.volume = soundVolume * 0.5 * volumeMultiplier;
      audio.play().catch((err) => {
        // Silently ignore autoplay errors
        if (err.name !== 'NotAllowedError') {
          console.warn(`Failed to play UI sound ${soundId}:`, err);
        }
      });
    },
    [soundEnabled, soundVolume]
  );

  const playComponentSound = useCallback(
    (componentId: string, volumeMultiplier: number = 1.0) => {
      if (!soundEnabled) return;

      const soundId = getComponentSound(componentId);
      const audio = componentSoundsRef.current[soundId];
      if (!audio) {
        // Fallback to generic select-part sound if component sound doesn't exist
        playSound('select-part', volumeMultiplier);
        return;
      }

      // Reset and play
      audio.currentTime = 0;
      audio.volume = soundVolume * 0.6 * volumeMultiplier;
      audio.play().catch((err) => {
        // Silently ignore autoplay errors
        if (err.name !== 'NotAllowedError') {
          console.warn(`Failed to play component sound ${soundId}:`, err);
        }
      });
    },
    [soundEnabled, soundVolume, playSound]
  );

  return { playSound, playComponentSound };
}
