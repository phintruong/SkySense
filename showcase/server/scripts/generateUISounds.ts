/** Regenerates UI sound MP3s with ElevenLabs. */
import 'dotenv/config';
import { generateSoundEffect } from '../src/services/soundEffects.js';
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

interface UISoundDefinition {
  id: string;
  name: string;
  prompt: string;
  duration: number;
  filename: string;
}

const UI_SOUNDS: UISoundDefinition[] = [
  {
    id: 'select-part',
    name: 'Select Part',
    prompt: 'short mechanical click, soft button press, UI selection sound',
    duration: 0.5,
    filename: 'select-part.mp3',
  },
  {
    id: 'hover-part',
    name: 'Hover Part',
    prompt: 'subtle soft whoosh, gentle air swoosh, light UI hover sound',
    duration: 0.5,
    filename: 'hover-part.mp3',
  },
  {
    id: 'attach-part',
    name: 'Attach Part',
    prompt: 'mechanical snap, parts clicking together, assembly sound, satisfying click',
    duration: 0.8,
    filename: 'attach-part.mp3',
  },
  {
    id: 'detach-part',
    name: 'Detach Part',
    prompt: 'mechanical release, parts separating, disassembly pop sound, metallic unclick',
    duration: 0.8,
    filename: 'detach-part.mp3',
  },
  {
    id: 'whoosh',
    name: 'Whoosh',
    prompt: 'quick air whoosh, fast swish sound, UI transition swoosh',
    duration: 0.6,
    filename: 'whoosh.mp3',
  },
  {
    id: 'success',
    name: 'Success',
    prompt: 'positive confirmation beep, successful action sound, pleasant UI feedback',
    duration: 0.7,
    filename: 'success.mp3',
  },
  {
    id: 'error',
    name: 'Error',
    prompt: 'gentle error tone, soft negative feedback, UI error sound',
    duration: 0.6,
    filename: 'error.mp3',
  },
];

async function generateAndSaveUISounds() {
  const outputDir = path.join(__dirname, '../../client/public/sounds/ui');

  // Create output directory if it doesn't exist
  if (!fs.existsSync(outputDir)) {
    fs.mkdirSync(outputDir, { recursive: true });
    console.log(`Created directory: ${outputDir}`);
  }

  console.log('\nGenerating UI sound effects...\n');

  for (const sound of UI_SOUNDS) {
    const outputPath = path.join(outputDir, sound.filename);

    // Skip if file already exists
    if (fs.existsSync(outputPath)) {
      console.log(`✓ ${sound.name} already exists, skipping...`);
      continue;
    }

    console.log(`Generating: ${sound.name}...`);
    console.log(`  Prompt: ${sound.prompt}`);
    console.log(`  Duration: ${sound.duration}s`);

    try {
      const audioBuffer = await generateSoundEffect(sound.prompt, sound.duration);

      if (!audioBuffer) {
        console.error(`✗ Failed to generate ${sound.name} (no API key or service error)`);
        continue;
      }

      fs.writeFileSync(outputPath, audioBuffer);
      console.log(`✓ Saved: ${sound.filename}\n`);

      // Wait 1 second between requests to avoid rate limiting
      await new Promise((resolve) => setTimeout(resolve, 1000));
    } catch (error) {
      console.error(`✗ Error generating ${sound.name}:`, error);
    }
  }

  console.log('\nUI sound generation complete!');
  console.log(`Sounds saved to: ${outputDir}\n`);
}

// Run the script
generateAndSaveUISounds().catch(console.error);
