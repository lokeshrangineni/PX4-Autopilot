#!/bin/bash
set -euo pipefail

# The image's default /root working directory is inaccessible to OpenShift's
# arbitrary UID. Ogre also resolves relative textures from the working directory.
mkdir -p /tmp/px4-demo-runtime
cd /tmp/px4-demo-runtime

# Match the local Mac demo's broad, pale grid and yellow X500 while retaining
# this profile's LiDAR-equipped vehicle dynamics.
python3 - <<'PY'
import struct
import zlib

size = 2000  # 500 m at 4 pixels per metre.
pixels = bytearray()
for y in range(size):
    pixels.append(0)
    for x in range(size):
        if x % 40 == 0 or y % 40 == 0:
            color = (65, 86, 108)
        elif x % 8 == 0 or y % 8 == 0:
            color = (100, 122, 144)
        else:
            color = (220, 228, 235)
        pixels.extend(color)

def chunk(kind, data):
    return (struct.pack("!I", len(data)) + kind + data
            + struct.pack("!I", zlib.crc32(kind + data)))

header = struct.pack("!2I5B", size, size, 8, 2, 0, 0, 0)
with open("light_grid.png", "wb") as image:
    image.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header)
                + chunk(b"IDAT", zlib.compress(pixels)) + chunk(b"IEND", b""))
PY

mkdir -p models/px4_demo_grid/materials/textures
cp light_grid.png models/px4_demo_grid/materials/textures/
cat > models/px4_demo_grid/model.config <<'EOF'
<?xml version="1.0"?>
<model>
  <name>PX4 demo grid</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
</model>
EOF
cat > models/px4_demo_grid/model.sdf <<'EOF'
<?xml version="1.0"?>
<sdf version="1.9"><model name="px4_demo_grid"/></sdf>
EOF

model_source="${PX4_GZ_MODELS}/x500_base"
mkdir -p models/x500_base
cp "${model_source}/model.config" models/x500_base/
cp "${model_source}/model.sdf" models/x500_base/
ln -sfn "${model_source}/meshes" models/x500_base/meshes
python3 - models/x500_base/model.sdf <<'PY'
import sys
import xml.etree.ElementTree as ET

path = sys.argv[1]
tree = ET.parse(path)
root = tree.getroot()
for visual in root.findall(".//visual"):
    old = visual.find("material")
    if old is not None:
        visual.remove(old)
    material = ET.SubElement(visual, "material")
    name = visual.get("name", "")
    if name == "base_link_visual":
        color = "1 0.8 0.02 1"
    elif "rotor" in name or "prop" in name:
        color = "1 0.15 0.02 1"
    else:
        color = "0.1 0.65 1 1"
    for field in ("ambient", "diffuse"):
        ET.SubElement(material, field).text = color
    ET.SubElement(material, "emissive").text = "0.12 0.08 0.01 1"

base = root.find(".//link[@name='base_link']")
base.append(ET.fromstring('''
  <visual name="demo_yellow_panel"><pose>0 0 0.075 0 0 0</pose>
    <geometry><box><size>0.18 0.22 0.04</size></box></geometry>
    <material><ambient>1 0.8 0 1</ambient><diffuse>1 0.8 0 1</diffuse>
      <emissive>0.3 0.2 0 1</emissive></material>
  </visual>'''))
tree.write(path, encoding="utf-8", xml_declaration=True)
PY

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
