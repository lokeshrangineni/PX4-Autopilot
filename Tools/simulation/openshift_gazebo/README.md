# PX4 Gazebo SITL on OpenShift

This example runs the PX4 `gz_x500` SITL example in OpenShift and provides a
TCP MAVLink bridge for a local QGroundControl client. It uses the PX4 Gazebo
Harmonic runtime image and the `baylands` world.

The manifest in `deployment.yaml` starts PX4, the Gazebo server, and
`mavlink-router` together in one Pod. The router exposes a TCP server on port
5760; use `oc port-forward` to reach it from the workstation. Gazebo GUI access
is a separate network problem and still requires a VPN or overlay with direct
reachability to the Pod's advertised Gazebo endpoints.

## Architecture

```text
Local workstation                         OpenShift Pod
┌────────────────────┐                    ┌─────────────────────────────┐
│ QGroundControl     │◄── TCP localhost ─►│ mavlink-router :5760        │
└────────────────────┘   oc port-forward  │       ↕ UDP 127.0.0.1       │
                                           │ PX4 :18570 + Gazebo server  │
                                           └─────────────────────────────┘
Gazebo GUI ◄──────────── VPN / overlay ────────────► Gazebo server
```

The router accepts QGroundControl's TCP connection and relays MAVLink to PX4
over Pod-local UDP. The workstation does not need a public UDP listener or an
external UDP load balancer. `oc port-forward` provides a temporary authenticated
TCP path to the Pod.

## Prerequisites

- `oc` is logged in to the target cluster and the active project is the
  intended sandbox project.
- The cluster can pull `px4io/px4-sitl-gazebo:v1.18.0-beta2` from Docker Hub, or the
  image has been mirrored to an approved registry.
- The project permits the Pod under its assigned restricted security context.
- OpenShift binary builds are enabled for the project.

## Deploy the headless SITL Pod

Build the small `mavlink-router` sidecar image from the upstream v4 source, then
apply the deployment:

```sh
oc project px4-loki-gazebo-demo
oc apply -f Tools/simulation/openshift_gazebo/mavlink-router-build.yaml
oc start-build mavlink-router \
  --from-dir=Tools/simulation/openshift_gazebo/mavlink-router --follow
oc apply -f Tools/simulation/openshift_gazebo/deployment.yaml
oc rollout status deployment/px4-gazebo-sitl
oc logs deployment/px4-gazebo-sitl -f
```

The example uses `HEADLESS=1`, `PX4_SIM_MODEL=gz_x500`, and
`PX4_GZ_WORLD=baylands`. PX4 state and logs are temporary and will be lost
when the Pod is replaced.

## Connect local QGroundControl

Start the supervised fixed-port forward and leave it running (requires Python 3
and an authenticated `oc`):

```sh
python3 Tools/simulation/openshift_gazebo/forward_qgc.py
```

Use an `oc` client matching the cluster release, available from the OpenShift
console's **Help → Command Line Tools** page. Pass `--oc /path/to/oc` to use
that client without replacing the workstation's installed version. An old
client can authenticate successfully while failing on streaming connections.

The helper uses the current context at startup and the `px4-loki-gazebo-demo`
namespace. It prefers WebSocket streaming, keeps a read-only telemetry monitor
connected, and restarts `oc port-forward` if the stream closes or receives no
data for 10 seconds. The monitor does not send flight commands or GCS
heartbeats. An expired login
stops the helper; refresh `oc login` and run it again. Use `--port` or
`--namespace` to override the defaults.

A direct temporary tunnel can also be started with:

```sh
KUBECTL_PORT_FORWARD_WEBSOCKETS=true oc port-forward \
  deployment/px4-gazebo-sitl 5760:5760 -n px4-loki-gazebo-demo
```

In QGroundControl, open **Settings → Comm Links**, add a **TCP** link, set the
host to `127.0.0.1` and port to `5760`, then connect. QGroundControl's UDP
auto-connect is not expected to discover a containerized PX4 automatically.

If QGC reports **The remote host closed the connection**, inspect the tunnel
terminal. An `error creating error stream ... Timeout occurred` message means
the tunnel failed before the connection reached the Pod. A local listener can
remain open in that state. Restart the direct tunnel, or let the helper recover
it, then reconnect the saved QGC link. Pod replacement also requires a fresh
tunnel; QGC may require a manual reconnect after recovery.

The port-forward carries TCP only. It does not provide access to Gazebo
Transport or expose a Gazebo GUI.

## LiDAR collision prevention (Position mode)

The optional profile replaces `gz_x500` with PX4's bundled
`gz_x500_lidar_2d` model and uses a lightweight synthetic obstacle course.
Gazebo feeds its horizontal laser scan directly into
PX4's `obstacle_distance` topic through `gz_bridge`; no ROS bridge is needed.
The startup script launches Gazebo with EGL headless rendering and Mesa software
rendering, so the GPU LiDAR sensor can run on a CPU-only cluster node.
It creates a temporary sensor-model override with 540 samples at 10 Hz rather
than the bundled 1080 samples at 30 Hz, preserving the field of view and range.
Software rendering can still run slower than real time; inspect Gazebo's
`/world/avoidance_course/stats` before flight.

Apply it after deploying the base example (this replaces the current simulated
vehicle and its temporary logs):

```sh
oc create configmap px4-collision-prevention-startup \
  --from-file=start_collision_prevention.sh=Tools/simulation/openshift_gazebo/start_collision_prevention.sh \
  --from-file=avoidance_course.sdf=Tools/simulation/openshift_gazebo/avoidance_course.sdf \
  -n px4-loki-gazebo-demo --dry-run=client -o yaml | oc apply -f -
oc patch deployment px4-gazebo-sitl -n px4-loki-gazebo-demo \
  --type=strategic \
  --patch-file Tools/simulation/openshift_gazebo/collision-prevention-patch.yaml
oc rollout status deployment/px4-gazebo-sitl -n px4-loki-gazebo-demo
```

Use the matching `oc` client described above. After editing an already deployed
startup ConfigMap, run `oc rollout restart deployment/px4-gazebo-sitl` in the
same namespace to restart the processes with the new script. To revert, inspect
`oc rollout history deployment/px4-gazebo-sitl` and undo to the revision that
used the original `gz_x500` profile, using `--to-revision` in the same namespace.

The course has a wall 15 m east of spawn (6 m high), and a building centred
20 m north of spawn (8 m high). These are known, simple collision obstacles.
Its geographic origin matches Baylands, but its geometry does **not** match
QGC satellite imagery. Baylands with software-rendered LiDAR ran at about
0.03 real-time factor on the current CPU node; use the course for initial
checks. Detailed scenes need rendering optimisation or a GPU-enabled node
and a separately configured hardware rendering profile.

The profile sets `CP_DIST=3` metres, `CP_DELAY=0.4` seconds,
`CP_GO_NO_DATA=0`, `CP_GUIDE_ANG=30` degrees, `MPC_POS_MODE=4` (acceleration
based), and `MPC_VEL_MANUAL=2` m/s. The delay is a starting value that needs
flight validation; setting parameters alone does not establish a safe stopping
distance. No flight or arming is performed by the startup script.

**Mode limitation:** PX4 collision prevention applies to manual acceleration
commands in **Position mode**, using a joystick or QGC virtual joysticks.
It does **not** protect QGC **Go to location**, Mission, Return, or arbitrary
Offboard commands. The scan covers 270 degrees horizontally; unseen directions
are blocked. It provides no vertical clearance sensing and does not plan routes
around buildings. Limited steering guidance is not autonomous path planning.
See the [PX4 collision prevention guide](https://docs.px4.io/main/en/computer_vision/collision_prevention).

After reconnecting QGC, confirm the parameter values and select Position mode
with manual input available. Before a flight, inspect the live scan:

```sh
oc exec deployment/px4-gazebo-sitl -n px4-loki-gazebo-demo -c px4-gazebo -- \
  /opt/px4-gazebo/bin/px4-listener obstacle_distance -n 2
```

The timestamp should advance and distances should contain valid measurements
rather than only `65535` (unknown). A no-return measurement beyond maximum range
is also valid. Pod readiness alone does not confirm that the renderer or sensor
is working. Start with a short, slow flight toward a known obstacle and confirm
that PX4 stops before the configured margin. This flight check must be completed
before treating the profile as validated. If scan data is lost, Position mode
restricts horizontal motion and subsequently switches to Hold.

Automatic rerouting for map-selected destinations requires a separately designed
companion planner that owns the trajectory and flight-mode transitions. PX4's
older mission path-planning interface is
[no longer supported](https://docs.px4.io/main/en/computer_vision/path_planning_interface).

## Automatic destinations in the synthetic obstacle course

The optional `course-planner` sidecar adds a local map for route previews and
explicitly starting a route. It uses A* on a 1 m grid, with 4.5 m obstacle
inflation, and reads the static, unrotated box geometry from the same course SDF
as Gazebo. It is a bounded, fixed-altitude simulation example, **not** a general
Baylands planner, SLAM system, or three-dimensional avoidance stack.

The native Gazebo scan is relayed over pod-local UDP by `sensor_bridge.py`.
This is necessary because PX4's MAVLink `OBSTACLE_DISTANCE` stream publishes
the Position-mode fused topic, which is not a fresh raw-sensor source in
Offboard mode. LiDAR supplies an independent stop check; it does not discover
new routes around unknown obstacles. New obstructions stop the route for operator
inspection rather than causing an automatic detour.

Build and apply this profile **after** the collision-prevention profile:

```sh
oc apply -f Tools/simulation/openshift_gazebo/planner-build.yaml -n px4-loki-gazebo-demo
oc start-build px4-course-planner -n px4-loki-gazebo-demo \
  --from-dir=Tools/simulation/openshift_gazebo/planner --follow
oc create configmap px4-collision-prevention-startup \
  --from-file=start_collision_prevention.sh=Tools/simulation/openshift_gazebo/start_collision_prevention.sh \
  --from-file=avoidance_course.sdf=Tools/simulation/openshift_gazebo/avoidance_course.sdf \
  -n px4-loki-gazebo-demo --dry-run=client -o yaml | oc apply -f -
oc create configmap px4-planner-sensor-bridge \
  --from-file=sensor_bridge.py=Tools/simulation/openshift_gazebo/planner/sensor_bridge.py \
  -n px4-loki-gazebo-demo --dry-run=client -o yaml | oc apply -f -
oc patch deployment px4-gazebo-sitl -n px4-loki-gazebo-demo --type=strategic \
  --patch-file Tools/simulation/openshift_gazebo/planner-patch.yaml
oc rollout status deployment/px4-gazebo-sitl -n px4-loki-gazebo-demo
oc port-forward deployment/px4-gazebo-sitl 8080:8080 -n px4-loki-gazebo-demo
```

Open `http://127.0.0.1:8080`. Keep the existing port-5760 QGC tunnel running as
well. Rebuilding an already deployed planner image or updating these ConfigMaps
requires a Deployment restart. The namespace in the image reference is specific
to this example project.

1. Connect QGC. Keep the simulated vehicle disarmed while inspecting route
   previews; no preview or telemetry request starts flight.
2. Select a destination on the **planner map** and click **Preview route**.
   North is up and east is right. Destinations outside ±40 m or inside inflated
   obstacles are rejected. A destination north 0 m, east 25 m lies behind the
   east wall and should produce a route around a wall end.
3. Arm and take off using QGC, then hover in Hold between 2 and 5 m above the
   course origin. The planner never arms or takes off.
4. Preview again, then click **Start automatic route**. The planner checks
   telemetry, airborne state, attitude, frame alignment, raw scan freshness,
   and the route before warming a 20 Hz setpoint stream and requesting Offboard.
   It preserves the initial altitude and limits horizontal speed to 1 m/s.
5. **Stop and hold** requests Hold. Switching mode in QGC cancels the route.
   Arrival also requests Hold; use QGC to land. No automatic landing is provided.

**QGC Go to location bypasses this planner.** Use the planner map for automatic
destinations. Mission/Return paths are also not planned by this example. QGC
continues to display telemetry and provide flight-mode, takeoff, and land controls.

The planner turns toward travel before moving, checks a forward travel cone
against the raw horizontal scan, and requests Hold if an obstacle is closer
than 3.5 m. Stale position, attitude, global position, or scan data stops the
route. Loss of the map's status polling for 3 seconds also stops it. Offboard
setpoint loss is configured to switch PX4 to Hold after 0.5 seconds. These are
implementation limits, not verified flight-safety guarantees. A process restart
does not resume or automatically begin a route.

The initial deployment must still be checked in flight for route tracking,
stopping clearance, operator takeover, sensor loss, and Offboard-loss behaviour.
Do not extend this known-course example to physical vehicles. General maps
require a maintained mapping/perception and planning stack, vertical sensing,
and a separate integration and validation effort.

The automatic profile uses `Recreate` deployment strategy: stop or finish a
simulated flight before changing it. It requests 1 CPU and 2 GiB for the course
simulator, with the existing 4 CPU / 8 GiB limits retained, plus the planner's
resources. This avoids requiring two simulator pods during an update on a busy
cluster. Replacement resets the simulated vehicle and temporary logs.

## Validation checklist

1. `oc rollout status` completes and the Pod remains Running.
2. Pod logs show PX4 SITL and Gazebo starting with the expected model/world.
3. `oc port-forward` binds the chosen local port to 5760 in the Pod.
4. QGroundControl's TCP Comm Link connects to `127.0.0.1:5760` and shows the
   simulated vehicle telemetry.
5. Replacing the Pod behaves as expected; this example intentionally has no
   persistent volume.

For a local Gazebo GUI, use a network overlay/VPN that provides direct Pod
endpoint reachability. Gazebo discovery alone is insufficient because peers
also connect to the advertised transport addresses.

## Cluster access status

On 2026-10-19 the collision-prevention profile rolled out with both containers
Ready and no container restarts. The lightweight course with the reduced-cost
LiDAR ran at approximately 1.0 real-time factor. Two consecutive
`obstacle_distance` messages advanced by 100 ms and measured the wall at about
14.4 m from the sensor. Accelerometer and gyro clipping counters were zero.
The configured collision-prevention parameters were read back from PX4.

The automatic course planner was subsequently built and deployed in the same
project with all three containers Running. Its live API reported fresh position,
attitude, global position, and raw LiDAR data. A route preview to north 0 m,
east 25 m detoured around the southern end of the east wall; local route checks
also rejected destinations inside the wall, outside the course, and non-finite
coordinates. The final runtime-speed sample was approximately 1.0 real-time
factor. The user then took off through QGC; the vehicle was about 2.5 m above
the origin while the planner remained idle in preview mode.

Automatic route flight, stopping clearance, operator takeover, and fault recovery
have **not** yet been verified in flight. The planner map port-forward uses
localhost 8080; the existing QGC tunnel uses localhost 5760. QGC needs to
reconnect after Pod replacement. Both port-forwards are temporary processes.
