# Sound Generation Quick Start

Generate the drift sound effect and other sounds for your robot viewer using ElevenLabs API.

## Setup (One-time)

1. **Get your ElevenLabs API key**
   - Sign up at https://elevenlabs.io/
   - Go to your profile settings and copy your API key

2. **Set the API key as environment variable**
   ```bash
   export ELEVENLABS_API_KEY="your-api-key-here"
   ```

   Or add it to your `.bashrc` or `.zshrc` for permanent use:
   ```bash
   echo 'export ELEVENLABS_API_KEY="your-api-key-here"' >> ~/.bashrc
   source ~/.bashrc
   ```

## Generate Drift Sound

The simplest way to generate the drift sound:

```bash
npm run generate-drift
```

This will create `public/sounds/ui/drift.mp3` with a 3-second tire screeching sound effect.

## Generate Custom Sounds

For other sound effects:

```bash
# Using npm script
USE_SOUND_EFFECTS=true DURATION=3 npm run generate-sound drift "car tire screeching"

# Or directly with the shell script
./scripts/generate-drift-sound.sh

# Or with the node script
USE_SOUND_EFFECTS=true DURATION=3 node scripts/generate-sound.js engine "car engine revving"
```

## Examples

```bash
# Generate a 2-second engine sound
USE_SOUND_EFFECTS=true DURATION=2 npm run generate-sound engine "race car engine revving at high RPM"

# Generate a 1-second brake sound
USE_SOUND_EFFECTS=true DURATION=1 npm run generate-sound brake "car brake squeal"

# Generate a boost sound
USE_SOUND_EFFECTS=true DURATION=2 npm run generate-sound boost "rocket booster whoosh sound effect"
```

## What Happens Next

1. The script calls ElevenLabs API with your description
2. Audio is generated and downloaded
3. File is saved to `public/sounds/ui/<sound-name>.mp3`
4. The sound is automatically available to your app (no restart needed in dev mode)

## Tips

- **Be specific**: "car tire screeching on wet asphalt" works better than "drift sound"
- **Keep it short**: 1-5 seconds is ideal for UI sound effects
- **Iterate**: If the first result isn't perfect, try rewording your description

## Troubleshooting

| Error | Solution |
|-------|----------|
| API key not set | Run `export ELEVENLABS_API_KEY="your-key"` |
| 401 Unauthorized | Check your API key is correct |
| 402 Payment Required | You've hit your API quota limit |
| File not found | Make sure you're in the client directory |

## Cost

ElevenLabs free tier includes:
- 10,000 characters per month
- Each sound generation typically uses ~50-100 characters
- You can generate ~100-200 sounds per month for free

## Full Documentation

See [scripts/README.md](scripts/README.md) for detailed documentation.
