#!/usr/bin/env bash
set -e

PROJECT="/mnt/c/Users/tommy/Desktop/Expo60/EXPO-60-UnderwaterDrone"
PI_USER="pi"
PI_HOST="192.168.137.50"
PI_PASS="raspberry"
REMOTE_PATH="/home/pi/torpedo_main"

echo "Building torpedo_main for BlueOS..."

cmake \
  -S "$PROJECT" \
  -B "$PROJECT/build-rpi" \
  -DCMAKE_TOOLCHAIN_FILE="$PROJECT/toolchain-rpi64.cmake"

cmake --build "$PROJECT/build-rpi" --parallel

echo "Uploading torpedo_main to Raspberry Pi..."

sshpass -p "$PI_PASS" scp \
  "$PROJECT/build-rpi/torpedo_main" \
  "$PI_USER@$PI_HOST:$REMOTE_PATH"

echo "Upload completed successfully."

if [ "$1" = "yes" ]; then
  echo "Starting torpedo_main on Raspberry Pi..."

  sshpass -p "$PI_PASS" ssh "$PI_USER@$PI_HOST" \
    "chmod +x $REMOTE_PATH && $REMOTE_PATH"
else
  echo "Program was not started."
fi