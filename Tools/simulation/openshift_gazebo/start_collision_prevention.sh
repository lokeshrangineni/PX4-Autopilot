#!/bin/bash
set -euo pipefail

# The image's default /root working directory is inaccessible to OpenShift's
# arbitrary UID. Ogre also resolves relative textures from the working directory.
mkdir -p /tmp/px4-demo-runtime
cd /tmp/px4-demo-runtime

# Keep the image's model untouched. Reduce scan cost for the CPU-only profile
# while preserving its 270-degree coverage and 30 m range.
sensor_source="${PX4_GZ_MODELS}/lidar_2d_v2"
mkdir -p models/lidar_2d_v2
cp "${sensor_source}/model.config" models/lidar_2d_v2/
ln -sfn "${sensor_source}/meshes" models/lidar_2d_v2/meshes
python3 - "$sensor_source/model.sdf" models/lidar_2d_v2/model.sdf <<'PY'
import sys
import xml.etree.ElementTree as ET

tree = ET.parse(sys.argv[1])
sensor = tree.find(".//sensor[@name='lidar_2d_v2']")
sensor.find("update_rate").text = "10"
sensor.find("ray/scan/horizontal/samples").text = "540"
tree.write(sys.argv[2], encoding="utf-8", xml_declaration=True)
PY
export GZ_SIM_RESOURCE_PATH="${PWD}/models:${GZ_SIM_RESOURCE_PATH}"

# GPU LiDAR still needs a renderer when the Gazebo GUI is disabled. Use EGL
# with Mesa software rendering on cluster nodes that do not expose a GPU.
gz sim --headless-rendering -r -s \
    "${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf" &
gazebo_pid=$!
px4_pid=""
bridge_pid=""
cleanup() {
    kill "$gazebo_pid" ${px4_pid:+"$px4_pid"} ${bridge_pid:+"$bridge_pid"} 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 143' TERM
trap 'exit 130' INT

# PX4 waits for the standalone server, spawns the LiDAR vehicle, and starts
# its native Gazebo-to-obstacle_distance bridge.
/opt/px4-gazebo/bin/px4 -d &
px4_pid=$!
if [ -f /etc/px4-planner/sensor_bridge.py ]; then
    python3 -u /etc/px4-planner/sensor_bridge.py &
    bridge_pid=$!
fi
# Restart the container if either process exits, including an unexpected
# successful exit. Neither a server-only nor a PX4-only container is useful.
wait -n "$gazebo_pid" "$px4_pid" ${bridge_pid:+"$bridge_pid"} || exit $?
exit 1
