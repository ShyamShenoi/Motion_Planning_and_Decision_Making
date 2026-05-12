#!/bin/bash
echo "=== Stopping CARLA server ==="
sudo docker stop carla-server 2>/dev/null || echo "No container named carla-server running"
sudo docker rm carla-server 2>/dev/null || echo "No container to remove"
echo "=== CARLA stopped ==="
