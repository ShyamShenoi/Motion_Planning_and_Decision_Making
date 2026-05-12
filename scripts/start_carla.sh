#!/bin/bash
set -e

echo "=== Allowing X11 connections for Docker ==="
xhost +local:docker

echo "=== Pulling CARLA 0.9.9.4 if not present ==="
sudo docker pull carlasim/carla:0.9.9.4

echo "=== Stopping any existing carla-server container ==="
sudo docker stop carla-server 2>/dev/null || true
sudo docker rm carla-server 2>/dev/null || true

echo "=== Starting CARLA server ==="
sudo docker run -d \
  --name carla-server \
  --runtime=nvidia \
  --privileged \
  --net=host \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  carlasim/carla:0.9.9.4 \
  /bin/bash -c "SDL_VIDEODRIVER=x11 ./CarlaUE4.sh \
    -opengl \
    -quality-level=Low \
    -nosound \
    -carla-rpc-port=2000 \
    -benchmark \
    -fps=20"

echo "=== CARLA is starting. Waiting 20 seconds for initialization... ==="
sleep 20
echo "=== CARLA server is ready. ==="
echo "=== Now run: bash scripts/build_and_run.sh ==="
