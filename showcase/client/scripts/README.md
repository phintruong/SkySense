# Sound Generation Scripts

Generate sound effects for the robot viewer using ElevenLabs API.

## Prerequisites

1. Get an ElevenLabs API key from https://elevenlabs.io/
2. Set it as an environment variable:
   ```bash
   export ELEVENLABS_API_KEY="your-api-key-here"
   ```

## Usage

### Generate Drift Sound

The easiest way to generate the drift sound:

```bash
chmod +x scripts/generate-drift-sound.sh
./scripts/generate-drift-sound.sh
```

### Generate Custom Sounds

For other sound effects, use the main script:

```bash
# Using Sound Effects API (recommended for sound effects)
USE_SOUND_EFFECTS=true DURATION=3 node scripts/generate-sound.js <sound-name> "<description>"

# Examples:
USE_SOUND_EFFECTS=true DURATION=3 node scripts/generate-sound.js drift "car tire screeching and drifting"
USE_SOUND_EFFECTS=true DURATION=2 node scripts/generate-sound.js engine "car engine revving up"
USE_SOUND_EFFECTS=true DURATION=1 node scripts/generate-sound.js click "mechanical click sound"
```

### Parameters

- `sound-name`: The name of the file (will be saved as `<sound-name>.mp3`)
- `description`: Text description of the sound you want to generate
- `USE_SOUND_EFFECTS`: Set to `true` to use Sound Effects API (better for actual sound effects)
- `DURATION`: Duration in seconds (default: 5)

## Current Sounds

The following sounds are used in the robot viewer:

- `select-part.mp3` - When selecting a robot part
- `hover-part.mp3` - When hovering over a part
- `attach-part.mp3` - When assembling parts
- `detach-part.mp3` - When disassembling parts
- `whoosh.mp3` - Transition sound
- `success.mp3` - Success notification
- `error.mp3` - Error notification
- `drift.mp3` - **NEW** - Drifting sound effect (needs to be generated)

## API Options

### Sound Effects API (Recommended)
Better for sound effects like drift, engine, etc.
```bash
USE_SOUND_EFFECTS=true DURATION=3 node scripts/generate-sound.js drift "description"
```

### Text-to-Speech API
Better for voice/speech, but can work for some sound effects:
```bash
node scripts/generate-sound.js soundname "description"
```

## Tips for Good Sound Effects

1. **Be specific**: "car tire screeching on asphalt" is better than "drift sound"
2. **Add context**: "racing car drifting at high speed"
3. **Duration**: Keep effects short (1-5 seconds) for better performance
4. **Iterate**: Try different descriptions if the first result isn't perfect

## Troubleshooting

**"ELEVENLABS_API_KEY environment variable not set"**
- Make sure you've exported your API key: `export ELEVENLABS_API_KEY="your-key"`

**"API returned status 401"**
- Your API key is invalid or expired. Check https://elevenlabs.io/

**"API returned status 402"**
- You've exceeded your API quota. Check your ElevenLabs account.

**Sound doesn't match expectation**
- Try different descriptions
- Adjust the duration
- Use the Sound Effects API instead of TTS API
