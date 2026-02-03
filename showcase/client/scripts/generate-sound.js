#!/usr/bin/env node

/**
 * Generate sound effects using ElevenLabs API
 *
 * Usage:
 *   node scripts/generate-sound.js <sound-name> <description>
 *
 * Example:
 *   node scripts/generate-sound.js drift "car tire screeching and drifting sound effect"
 *
 * Requires ELEVENLABS_API_KEY environment variable
 */

const fs = require('fs');
const path = require('path');
const https = require('https');

// Configuration
const ELEVENLABS_API_KEY = process.env.ELEVENLABS_API_KEY;
const OUTPUT_DIR = path.join(__dirname, '../public/sounds/ui');

// Sound effects voice ID (use a voice optimized for sound effects)
// You can get this from ElevenLabs dashboard or use text-to-sound API
const VOICE_ID = 'EXAVITQu4vr4xnSDxMaL'; // Sarah voice - you may want to change this

async function generateSound(soundName, description) {
  if (!ELEVENLABS_API_KEY) {
    console.error('Error: ELEVENLABS_API_KEY environment variable not set');
    console.error('Get your API key from: https://elevenlabs.io/');
    process.exit(1);
  }

  if (!soundName || !description) {
    console.error('Usage: node generate-sound.js <sound-name> <description>');
    console.error('Example: node generate-sound.js drift "car tire screeching"');
    process.exit(1);
  }

  console.log(`Generating sound: ${soundName}`);
  console.log(`Description: ${description}`);
  console.log('Using ElevenLabs API...\n');

  // Ensure output directory exists
  if (!fs.existsSync(OUTPUT_DIR)) {
    fs.mkdirSync(OUTPUT_DIR, { recursive: true });
  }

  const outputPath = path.join(OUTPUT_DIR, `${soundName}.mp3`);

  // ElevenLabs Text-to-Speech API
  // Note: For sound effects, you might want to use their Sound Effects API instead
  // This uses TTS API - adjust the text to get the desired sound effect
  const options = {
    hostname: 'api.elevenlabs.io',
    path: `/v1/text-to-speech/${VOICE_ID}`,
    method: 'POST',
    headers: {
      'Accept': 'audio/mpeg',
      'Content-Type': 'application/json',
      'xi-api-key': ELEVENLABS_API_KEY
    }
  };

  const requestBody = JSON.stringify({
    text: description,
    model_id: 'eleven_monolingual_v1',
    voice_settings: {
      stability: 0.5,
      similarity_boost: 0.5
    }
  });

  return new Promise((resolve, reject) => {
    const req = https.request(options, (res) => {
      if (res.statusCode !== 200) {
        let errorBody = '';
        res.on('data', (chunk) => {
          errorBody += chunk.toString();
        });
        res.on('end', () => {
          console.error(`Error: API returned status ${res.statusCode}`);
          console.error(errorBody);
          reject(new Error(`API error: ${res.statusCode}`));
        });
        return;
      }

      const writeStream = fs.createWriteStream(outputPath);
      res.pipe(writeStream);

      writeStream.on('finish', () => {
        writeStream.close();
        console.log(`✓ Sound generated successfully!`);
        console.log(`✓ Saved to: ${outputPath}`);
        const stats = fs.statSync(outputPath);
        console.log(`✓ File size: ${(stats.size / 1024).toFixed(2)} KB`);
        resolve();
      });

      writeStream.on('error', (err) => {
        fs.unlink(outputPath, () => {});
        reject(err);
      });
    });

    req.on('error', (err) => {
      console.error('Request error:', err);
      reject(err);
    });

    req.write(requestBody);
    req.end();
  });
}

// Alternative: Use ElevenLabs Sound Effects API
async function generateSoundEffect(soundName, description, duration = 5) {
  if (!ELEVENLABS_API_KEY) {
    console.error('Error: ELEVENLABS_API_KEY environment variable not set');
    process.exit(1);
  }

  console.log(`Generating sound effect: ${soundName}`);
  console.log(`Description: ${description}`);
  console.log(`Duration: ${duration}s`);
  console.log('Using ElevenLabs Sound Effects API...\n');

  if (!fs.existsSync(OUTPUT_DIR)) {
    fs.mkdirSync(OUTPUT_DIR, { recursive: true });
  }

  const outputPath = path.join(OUTPUT_DIR, `${soundName}.mp3`);

  const options = {
    hostname: 'api.elevenlabs.io',
    path: '/v1/sound-generation',
    method: 'POST',
    headers: {
      'Accept': 'audio/mpeg',
      'Content-Type': 'application/json',
      'xi-api-key': ELEVENLABS_API_KEY
    }
  };

  const requestBody = JSON.stringify({
    text: description,
    duration_seconds: duration,
    prompt_influence: 0.3
  });

  return new Promise((resolve, reject) => {
    const req = https.request(options, (res) => {
      if (res.statusCode !== 200) {
        let errorBody = '';
        res.on('data', (chunk) => {
          errorBody += chunk.toString();
        });
        res.on('end', () => {
          console.error(`Error: API returned status ${res.statusCode}`);
          console.error(errorBody);
          reject(new Error(`API error: ${res.statusCode}`));
        });
        return;
      }

      const writeStream = fs.createWriteStream(outputPath);
      res.pipe(writeStream);

      writeStream.on('finish', () => {
        writeStream.close();
        console.log(`✓ Sound effect generated successfully!`);
        console.log(`✓ Saved to: ${outputPath}`);
        const stats = fs.statSync(outputPath);
        console.log(`✓ File size: ${(stats.size / 1024).toFixed(2)} KB`);
        resolve();
      });

      writeStream.on('error', (err) => {
        fs.unlink(outputPath, () => {});
        reject(err);
      });
    });

    req.on('error', (err) => {
      console.error('Request error:', err);
      reject(err);
    });

    req.write(requestBody);
    req.end();
  });
}

// Main execution
const args = process.argv.slice(2);
const soundName = args[0];
const description = args.slice(1).join(' ');
const useSoundEffects = process.env.USE_SOUND_EFFECTS === 'true';

if (useSoundEffects) {
  // Use Sound Effects API (better for actual sound effects)
  const duration = parseInt(process.env.DURATION) || 5;
  generateSoundEffect(soundName, description, duration)
    .catch((err) => {
      console.error('Failed to generate sound:', err);
      process.exit(1);
    });
} else {
  // Use TTS API (may not work well for sound effects)
  generateSound(soundName, description)
    .catch((err) => {
      console.error('Failed to generate sound:', err);
      process.exit(1);
    });
}
