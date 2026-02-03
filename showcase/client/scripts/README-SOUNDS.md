# Component Sound Generation

This directory contains a script to generate unique sounds for each robot component using the ElevenLabs API.

## What's Been Implemented

The robot viewer now plays **unique sounds for each component type** when you click on them:

- **Wheels** (`wheel-1`, `wheel-2`): Tire rolling sound
- **IR Sensors** (`sensor-1`, `sensor-2`): Electronic beep/ping
- **Ultrasonic Sensor** (`ultrasonic-sensor-1`): Sonar echo ping
- **Color Sensor** (`color-sensor-1`): Digital scanning beep
- **Arduino** (`arduino-1`): Microcontroller boot chirp
- **Motor Driver** (`motor-driver-1`): Electric motor whir
- **Batteries** (`battery-1`, `battery-2`): Power on hum
- **Breadboard** (`breadboard-1`): Circuit click
- **Base Plate** (`plate-1`): Metallic click

## How to Generate Sounds

### 1. Get an ElevenLabs API Key

Sign up at [elevenlabs.io](https://elevenlabs.io/) and get your API key from the account settings.

### 2. Set Your API Key

```bash
export ELEVENLABS_API_KEY=your_api_key_here
```

### 3. Run the Generation Script

```bash
cd /Users/notjackl3/Desktop/CODING/UTRA-Hacks/showcase/robot-viewer/client
npm run generate-component-sounds
```

The script will:
- Generate 9 unique MP3 sound files (one for each component type)
- Save them to `public/sounds/components/`
- Add a 1-second delay between each generation to avoid rate limiting
- Show progress and success/failure status for each sound

### 4. Sounds Will Be Automatically Loaded

Once generated, the sounds will be automatically loaded by the app and played when you click on components. If a sound file doesn't exist, the app will fallback to the generic `select-part.mp3` sound.

## Generated Sound Files

The following files will be created in `public/sounds/components/`:

- `wheel.mp3` - Rubber tire rolling on pavement
- `sensor.mp3` - High-pitched electronic sensor ping
- `ultrasonic-sensor.mp3` - Ultrasonic sonar echo
- `color-sensor.mp3` - Digital scanning beep
- `arduino.mp3` - Computer processor boot sound
- `motor-driver.mp3` - Electric motor whir
- `battery.mp3` - Electrical power on sound
- `breadboard.mp3` - Electronic circuit click
- `plate.mp3` - Metallic structural click

## Technical Details

- **Sound Duration**: ~1-1.5 seconds each
- **Format**: MP3
- **Volume**: Component sounds play at 60% of global volume
- **Fallback**: If a component sound doesn't exist, falls back to generic `select-part.mp3`
- **API Used**: ElevenLabs Sound Effects API endpoint

## Troubleshooting

**"ELEVENLABS_API_KEY environment variable is not set"**
- Make sure you've exported your API key in the terminal session where you're running the command

**"ElevenLabs API error (401)"**
- Your API key is invalid or expired. Get a new one from elevenlabs.io

**"ElevenLabs API error (429)"**
- You've hit the rate limit. Wait a few minutes and try again, or upgrade your ElevenLabs plan

**Sound files generated but not playing**
- Clear your browser cache and reload the page
- Check the browser console for any audio loading errors
- Verify the files exist in `client/public/sounds/components/`
