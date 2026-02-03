/** Sounds: list engine sounds, generate audio, serve UI sounds. */
import { Router } from 'express';
import { getAvailableSounds, getSoundById, generateSoundEffect, getAvailableUISounds, getUISoundById } from '../services/soundEffects.js';

const router = Router();

// Get all available engine sounds
router.get('/', (_req, res) => {
  const sounds = getAvailableSounds();
  res.json({ sounds });
});

// Get a specific sound by ID
router.get('/:id', (req, res) => {
  const sound = getSoundById(req.params.id);
  if (!sound) {
    return res.status(404).json({ error: 'Sound not found' });
  }
  res.json({ sound });
});

// Generate sound effect audio
router.post('/generate/:id', async (req, res) => {
  const sound = getSoundById(req.params.id);
  if (!sound) {
    return res.status(404).json({ error: 'Sound not found' });
  }

  const duration = req.body.duration || 2;
  const audioBuffer = await generateSoundEffect(sound.prompt, duration);

  if (!audioBuffer) {
    return res.status(503).json({
      error: 'Sound generation unavailable',
      message: 'Sound generation is not available'
    });
  }

  res.set({
    'Content-Type': 'audio/mpeg',
    'Content-Length': audioBuffer.length,
  });
  res.send(audioBuffer);
});

// Generate custom sound effect
router.post('/generate-custom', async (req, res) => {
  const { prompt, duration = 2 } = req.body;

  if (!prompt) {
    return res.status(400).json({ error: 'Prompt is required' });
  }

  const audioBuffer = await generateSoundEffect(prompt, duration);

  if (!audioBuffer) {
    return res.status(503).json({
      error: 'Sound generation unavailable',
      message: 'Sound generation is not available'
    });
  }

  res.set({
    'Content-Type': 'audio/mpeg',
    'Content-Length': audioBuffer.length,
  });
  res.send(audioBuffer);
});

// Get all available UI sounds
router.get('/ui', (_req, res) => {
  const sounds = getAvailableUISounds();
  res.json({ sounds });
});

// Get a specific UI sound by ID
router.get('/ui/:id', (req, res) => {
  const sound = getUISoundById(req.params.id);
  if (!sound) {
    return res.status(404).json({ error: 'UI sound not found' });
  }
  res.json({ sound });
});

export default router;
