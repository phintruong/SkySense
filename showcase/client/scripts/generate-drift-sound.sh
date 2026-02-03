#!/bin/bash

# Generate drift sound effect using ElevenLabs API
# Make sure to set ELEVENLABS_API_KEY environment variable first

echo "Generating drift sound effect..."
echo ""

# Use Sound Effects API for better results
USE_SOUND_EFFECTS=true DURATION=3 node scripts/generate-sound.js drift "car tire screeching and drifting on asphalt, racing car skidding sound effect"

echo ""
echo "Done! The drift sound is now available at: public/sounds/ui/drift.mp3"
