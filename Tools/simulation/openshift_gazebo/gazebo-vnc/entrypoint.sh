#!/usr/bin/env bash
set -euo pipefail

export HOME="${HOME:-/tmp/gazebo-vnc-home}"
export XDG_CONFIG_HOME="${XDG_CONFIG_HOME:-$HOME/.config}"
export XDG_CACHE_HOME="${XDG_CACHE_HOME:-/tmp/gazebo-vnc-cache}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-$(id -u)}"
export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1

mkdir -p "$HOME" "$XDG_CONFIG_HOME" "$XDG_CACHE_HOME" "$XDG_RUNTIME_DIR" /tmp/gazebo-vnc
chmod 0700 "$XDG_RUNTIME_DIR"
cd "$HOME"

Xvfb "$DISPLAY" -screen 0 1600x900x24 -nolisten tcp &
x_server_pid=$!
gui_pid=
vnc_pid=
websockify_pid=

cleanup() {
	kill "$websockify_pid" "$vnc_pid" "$gui_pid" "$x_server_pid" 2>/dev/null || true
	wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

for attempt in $(seq 1 50); do
	if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
		break
	fi
	sleep 0.2
done
if ! xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
	echo "Xvfb did not become ready" >&2
	exit 1
fi

# The PX4 pod runs the Gazebo server in headless mode. Join its transport
# partition with a GUI-only client; do not start another simulation world.
for attempt in $(seq 1 450); do
	if [[ -f /tmp/models/px4_demo_grid/materials/textures/light_grid.png ]]; then
		break
	fi
	sleep 0.2
done
if [[ ! -f /tmp/models/px4_demo_grid/materials/textures/light_grid.png ]]; then
	echo "The shared Gazebo grid assets did not become ready" >&2
	exit 1
fi
gz sim -g --gui-config /usr/local/share/gazebo-vnc/gui.config &
gui_pid=$!
x11vnc -display "$DISPLAY" -nopw -rfbport 5901 -localhost \
	-forever -shared -noxdamage &
vnc_pid=$!
websockify --web=/usr/share/novnc/ 6080 127.0.0.1:5901 &
websockify_pid=$!

wait -n "$x_server_pid" "$gui_pid" "$vnc_pid" "$websockify_pid"
echo "A Gazebo VNC process exited" >&2
exit 1
