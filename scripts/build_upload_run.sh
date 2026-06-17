#!/usr/bin/env bash
set -euo pipefail

PROJECT_WSL="/mnt/c/Users/tommy/Desktop/Expo60/EXPO-60-UnderwaterDrone"

PI_USER="pi"
PI_HOST="192.168.137.50"

REMOTE_PROJECT_ROOT="/home/pi/torpedo_EX60_Main"
REMOTE_BLUEOS_PATH="$REMOTE_PROJECT_ROOT/BlueOs"

RUN_AFTER_UPLOAD="${1:-no}"
DOCKER_AFTER_UPLOAD="${2:-no}"

DOCKER_IMAGE="smart-control-allocation:0.0.1"
DOCKER_CONTAINER="smart-control-allocation"

echo "=== Building locally for validation ==="

cmake \
  -S "$PROJECT_WSL" \
  -B "$PROJECT_WSL/build-rpi" \
  -DCMAKE_TOOLCHAIN_FILE="$PROJECT_WSL/toolchain-rpi64.cmake"

cmake --build "$PROJECT_WSL/build-rpi" --parallel

echo "=== Testing Windows SSH connection to Raspberry Pi ==="
powershell.exe -NoProfile -Command "ssh $PI_USER@$PI_HOST 'echo SSH_OK'"

echo "=== Preparing remote project directory ==="
powershell.exe -NoProfile -Command "ssh $PI_USER@$PI_HOST 'mkdir -p $REMOTE_PROJECT_ROOT && sudo chown -R $PI_USER:$PI_USER $REMOTE_PROJECT_ROOT'"

echo "=== Syncing project to Raspberry Pi ==="
tar \
  --exclude='.git' \
  --exclude='build' \
  --exclude='build-rpi' \
  --exclude='.vscode' \
  --exclude='BlueOs/__pycache__' \
  --exclude='BlueOs/.venv' \
  --exclude='BlueOs/data' \
  -C "$PROJECT_WSL" \
  -cf - . | powershell.exe -NoProfile -Command "ssh $PI_USER@$PI_HOST 'tar -xf - -C $REMOTE_PROJECT_ROOT'"

echo "=== Project sync completed successfully ==="

if [ "$DOCKER_AFTER_UPLOAD" = "yes" ]; then
  echo "=== Stopping existing Docker container if present ==="
  powershell.exe -NoProfile -Command "ssh $PI_USER@$PI_HOST 'sudo docker rm -f $DOCKER_CONTAINER >/dev/null 2>&1 || true'"

  echo "=== Building BlueOs Docker image on Raspberry Pi ==="
  powershell.exe -NoProfile -Command "ssh $PI_USER@$PI_HOST 'cd $REMOTE_BLUEOS_PATH && sudo docker build -t $DOCKER_IMAGE .'"

  echo "=== Starting BlueOs Docker container on Raspberry Pi ==="
  powershell.exe -NoProfile -Command "ssh $PI_USER@$PI_HOST 'sudo docker run -d --name $DOCKER_CONTAINER --network host -v $REMOTE_PROJECT_ROOT:/workspace -v $REMOTE_BLUEOS_PATH/data:/data $DOCKER_IMAGE'"

  echo "=== BlueOs Docker rebuild and start completed ==="
else
  echo "=== Docker step skipped ==="
fi

if [ "$RUN_AFTER_UPLOAD" = "yes" ]; then
  echo "=== Stopping any old torpedo_main on Raspberry Pi ==="
  powershell.exe -NoProfile -Command "ssh $PI_USER@$PI_HOST 'pkill -x torpedo_main >/dev/null 2>&1 || true'"

  echo "=== Removing old remote build folder ==="
  powershell.exe -NoProfile -Command "ssh $PI_USER@$PI_HOST 'rm -rf $REMOTE_PROJECT_ROOT/build'"

  echo "=== Building newest torpedo_main natively on Raspberry Pi ==="
  powershell.exe -NoProfile -Command "ssh $PI_USER@$PI_HOST 'cd $REMOTE_PROJECT_ROOT && cmake -S . -B build && cmake --build build -j\$(nproc)'"

  echo "=== Starting newest torpedo_main in foreground ==="
  powershell.exe -NoProfile -Command "ssh $PI_USER@$PI_HOST 'cd $REMOTE_PROJECT_ROOT && exec ./build/torpedo_main'"
else
  echo "=== torpedo_main was not started ==="
fi