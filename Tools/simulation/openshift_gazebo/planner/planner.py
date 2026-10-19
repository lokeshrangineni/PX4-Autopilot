#!/usr/bin/env python3
"""Bounded, fixed-altitude route-planning example for the synthetic SITL course.

Uses the world's known static box geometry, not SLAM or satellite imagery.
Never arms, takes off, or interprets QGC Go-to commands. Only /api/start may
begin Offboard control, and only when the simulated vehicle is already flying.
"""

import heapq
import json
import logging
import math
import os
from pathlib import Path
import queue
import socket
import threading
import time
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil

M = mavutil.mavlink
MAP_LIMIT = 40  # metres north/east of the Gazebo spawn
MARGIN = 4.5  # metres, includes vehicle dimensions and tracking margin
STOP_DISTANCE = 3.5  # metres, leaves room before the static route boundary
SPEED = 1.0  # m/s maximum horizontal route speed
FRESH = 0.5  # seconds, telemetry and raw scan deadline
TICK = 0.05  # 20 Hz setpoints
EARTH_RADIUS = 6371000.0
VELOCITY_MASK = (M.POSITION_TARGET_TYPEMASK_X_IGNORE |
                 M.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                 M.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                 M.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                 M.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                 M.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                 M.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)


def angle(value):
    return (value + math.pi) % (2 * math.pi) - math.pi


class Course:
    def __init__(self, filename):
        world = ET.parse(filename).getroot().find("world")
        if world.attrib["name"] != "avoidance_course":
            raise ValueError("Planner supports only avoidance_course")
        coords = world.find("spherical_coordinates")
        self.origin = [float(coords.findtext(key)) for key in
                       ("latitude_deg", "longitude_deg", "elevation")]
        self.boxes = []
        for model in world.findall("model"):
            if model.attrib["name"] == "ground":
                continue
            pose = [float(v) for v in model.findtext("pose").split()]
            size = model.findtext("link/collision/geometry/box/size")
            if size is None or any(abs(v) > 1e-6 for v in pose[3:]):
                raise ValueError("Course requires unrotated box obstacles")
            east_size, north_size, _ = map(float, size.split())
            east, north = pose[:2]
            self.boxes.append({"name": model.attrib["name"],
                               "n": north, "e": east,
                               "dn": north_size / 2, "de": east_size / 2})

    def free(self, point):
        n, e = point
        return (abs(n) <= MAP_LIMIT and abs(e) <= MAP_LIMIT and
                all(abs(n - box["n"]) > box["dn"] + MARGIN or
                    abs(e - box["e"]) > box["de"] + MARGIN
                    for box in self.boxes))

    def segment_free(self, start, end):
        steps = max(1, math.ceil(math.dist(start, end) / 0.2))
        return all(self.free(tuple(start[k] + (end[k] - start[k]) * i / steps
                                   for k in range(2))) for i in range(steps + 1))

    def route(self, start, goal):
        if not all(math.isfinite(v) for v in (*start, *goal)):
            raise ValueError("Coordinates must be finite")
        if not self.free(start) or not self.free(goal):
            raise ValueError("Start or destination is outside the course or obstacle margin")
        first, last = tuple(map(round, start)), tuple(map(round, goal))
        if not self.segment_free(start, first) or not self.segment_free(last, goal):
            raise ValueError("Endpoint is too close to an obstacle for the 1 m grid")
        frontier, cost, parent = [(0, first)], {first: 0}, {}
        while frontier:
            _, node = heapq.heappop(frontier)
            if node == last:
                path = [last]
                while path[-1] != first:
                    path.append(parent[path[-1]])
                path.reverse()
                # Compress only collinear segments. Keep all corners so that
                # the route does not cut across inflated obstacles.
                result = [list(start)]
                for i, point in enumerate(path):
                    if i == 0 or i == len(path) - 1 or (
                        point[0] - path[i-1][0], point[1] - path[i-1][1]
                    ) != (path[i+1][0] - point[0], path[i+1][1] - point[1]):
                        result.append(list(point))
                result.append(list(goal))
                if not all(self.segment_free(a, b) for a, b in zip(result, result[1:])):
                    raise ValueError("Route failed obstacle clearance check")
                return result
            for dn, de in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbor = node[0] + dn, node[1] + de
                score = cost[node] + 1
                if self.free(neighbor) and score < cost.get(neighbor, math.inf):
                    cost[neighbor], parent[neighbor] = score, node
                    priority = score + abs(neighbor[0] - last[0]) + abs(neighbor[1] - last[1])
                    heapq.heappush(frontier, (priority, neighbor))
        raise ValueError("No route within the bounded course")


class Planner:
    def __init__(self, course):
        self.course = course
        self.link = mavutil.mavlink_connection(
            "udpin:127.0.0.1:14540", source_system=245,
            source_component=M.MAV_COMP_ID_ONBOARD_COMPUTER, dialect="common")
        self.scan_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.scan_socket.bind(("127.0.0.1", 14541))
        self.scan_socket.setblocking(False)
        self.lock, self.commands = threading.RLock(), queue.Queue()
        self.messages, self.seen = {}, {}
        self.scan = None
        self.path, self.phase, self.reason = [], "idle", "Waiting for simulator telemetry"
        self.index, self.height, self.started = 0, 0, 0
        self.entry_mode = None
        self.client_seen = 0
        self.target_system = 1
        self.last_boot_ms = 0

    def mode(self):
        hb = self.messages.get("HEARTBEAT")
        return (hb.custom_mode >> 16) & 255 if hb else 0

    def issue(self, command, *params):
        self.link.mav.command_long_send(self.target_system, 1, command, 0,
                                        *(list(params) + [0] * (7 - len(params))))

    def hold(self, reason):
        if self.phase != "idle" and self.mode() == 6:
            # Stop streaming after the Hold request. PX4's Offboard-loss
            # fallback is also configured to Hold if the request is lost.
            self.issue(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 3)
        self.phase, self.reason = "idle", reason
        logging.info(reason)

    def ready(self):
        now = time.monotonic()
        for kind in ("HEARTBEAT", "LOCAL_POSITION_NED", "ATTITUDE", "GLOBAL_POSITION_INT"):
            limit = 2.5 if kind == "HEARTBEAT" else FRESH
            if now - self.seen.get(kind, -math.inf) > limit:
                raise ValueError(f"Missing or stale {kind}")
        if self.scan is None or now - self.scan["observed"] > FRESH:
            raise ValueError("Missing or stale raw LiDAR")
        hb, p = self.messages["HEARTBEAT"], self.messages["LOCAL_POSITION_NED"]
        a, g = self.messages["ATTITUDE"], self.messages["GLOBAL_POSITION_INT"]
        if hb.autopilot != M.MAV_AUTOPILOT_PX4:
            raise ValueError("Expected PX4 SITL")
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, p.vx, p.vy, p.vz, a.roll, a.pitch, a.yaw)):
            raise ValueError("Invalid position or attitude")
        if abs(a.roll) > 0.4 or abs(a.pitch) > 0.4:
            raise ValueError("Vehicle attitude is unstable")
        # Check that the estimated local frame still matches this world's map.
        lat, lon, altitude = self.course.origin
        global_n = math.radians(g.lat / 1e7 - lat) * EARTH_RADIUS
        global_e = math.radians(g.lon / 1e7 - lon) * EARTH_RADIUS * math.cos(math.radians(lat))
        if math.hypot(global_n - p.x, global_e - p.y) > 2:
            raise ValueError("Local frame does not match course origin")
        if abs(g.alt / 1000 + p.z - altitude) > 2:
            raise ValueError("Altitude frame does not match course origin")
        return p

    def perform(self, action, payload):
        if action == "stop":
            self.hold("Stopped by operator")
            return {"ok": True}
        if self.phase != "idle":
            raise ValueError("Stop the active route before selecting another destination")
        p = self.ready()
        goal = [float(payload["north"]), float(payload["east"])]
        path = self.course.route([p.x, p.y], goal)
        if action == "preview":
            self.path, self.reason = path, "Route preview; not flying"
            return {"path": path}
        hb = self.messages["HEARTBEAT"]
        landed = self.messages.get("EXTENDED_SYS_STATE")
        if not hb.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED:
            raise ValueError("Arm and take off using QGC first; planner never arms")
        if (landed is None or time.monotonic() - self.seen.get("EXTENDED_SYS_STATE", 0) > 2.5
                or landed.landed_state != M.MAV_LANDED_STATE_IN_AIR):
            raise ValueError("Vehicle is not confirmed airborne")
        if not 2 <= -p.z <= 5 or abs(p.vz) > 0.3 or math.hypot(p.vx, p.vy) > 0.3:
            raise ValueError("Hover steadily between 2 and 5 m above the course origin")
        if self.mode() not in (3, 4):
            raise ValueError("Begin from Position or Hold mode")
        if self.mode() == 4 and (hb.custom_mode >> 24) & 255 != 3:
            raise ValueError("Begin from Hold, not another automatic mode")
        self.path, self.index, self.height = path, 1, p.z
        self.started, self.phase = time.monotonic(), "warming"
        self.reason, self.entry_mode = "Preparing Offboard control", hb.custom_mode
        logging.info("Route started: %s", path)
        return {"path": path, "ok": True}

    def view(self):
        with self.lock:
            self.client_seen = time.monotonic()
            p = self.messages.get("LOCAL_POSITION_NED")
            a = self.messages.get("ATTITUDE")
            hb = self.messages.get("HEARTBEAT")
            try:
                self.ready()
                telemetry_error = None
                if self.reason == "Waiting for simulator telemetry":
                    self.reason = "Ready for route preview"
            except ValueError as error:
                telemetry_error = str(error)
            return {"phase": self.phase, "reason": self.reason, "path": self.path,
                    "index": self.index, "telemetry_error": telemetry_error,
                    "position": [p.x, p.y, p.z] if p else None,
                    "yaw": a.yaw if a else 0, "mode": self.mode(),
                    "armed": bool(hb and hb.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED),
                    "boxes": self.course.boxes, "margin": MARGIN, "limit": MAP_LIMIT,
                    "origin": self.course.origin}

    def receive(self):
        for _ in range(100):
            msg = self.link.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_srcComponent() != 1 or msg.get_srcSystem() != self.target_system:
                continue
            kind = msg.get_type()
            if kind == "BAD_DATA":
                continue
            if kind == "LOCAL_POSITION_NED":
                if msg.time_boot_ms < self.last_boot_ms:
                    self.hold("PX4 restarted; select a new route after recovery")
                    self.messages.clear()
                    self.seen.clear()
                self.last_boot_ms = msg.time_boot_ms
            self.messages[kind], self.seen[kind] = msg, time.monotonic()
        for _ in range(100):
            try:
                data, _ = self.scan_socket.recvfrom(60000)
            except BlockingIOError:
                break
            scan = json.loads(data)
            if (0 <= time.monotonic() - scan["observed"] < FRESH
                    and len(scan["ranges"]) >= 100):
                self.scan = scan

    def clear_cone(self, bearing):
        # Gazebo rays are FLU (positive left); MAVLink attitude is NED/FRD.
        relative = angle(bearing - self.messages["ATTITUDE"].yaw)
        scan = self.scan
        distances = [value for i, value in enumerate(scan["ranges"])
                     if abs(angle(-scan["angle_min"] - i * scan["angle_step"] - relative)) < 0.8]
        if len(distances) < 20 or any(value is None for value in distances):
            raise ValueError("Travel direction has insufficient LiDAR coverage")
        # Includes >3 seconds of clearance at this demo's 1 m/s speed.
        if min(distances) < STOP_DISTANCE:
            raise ValueError("LiDAR obstacle in travel corridor; holding")

    def control(self):
        if self.phase == "idle":
            return
        hb = self.messages.get("HEARTBEAT")
        if (hb and time.monotonic() - self.seen["HEARTBEAT"] < 2.5 and
                ((self.phase == "flying" and self.mode() != 6) or
                 (self.phase in ("warming", "entering") and self.mode() != 6
                  and hb.custom_mode != self.entry_mode))):
            self.phase, self.reason = "idle", "Operator or failsafe changed flight mode"
            return
        try:
            p = self.ready()
            if time.monotonic() - self.client_seen > 3:
                raise ValueError("Planner map disconnected; holding")
            if not self.messages["HEARTBEAT"].base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED:
                raise ValueError("Vehicle disarmed")
            if not self.course.free([p.x, p.y]):
                raise ValueError("Vehicle outside planned obstacle clearance")
            if abs(p.z - self.height) > 0.8:
                raise ValueError("Altitude tracking error; holding")
            yaw = self.messages["ATTITUDE"].yaw
            vx = vy = vz = 0.0
            now = time.monotonic()
            if self.phase == "warming" and now - self.started >= 2:
                self.issue(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6)
                self.phase = "entering"
            if self.phase == "entering":
                if self.mode() == 6:
                    self.phase, self.reason = "flying", "Following planned route"
                elif now - self.started > 6:
                    raise ValueError("PX4 did not accept Offboard mode")
            if self.phase == "flying":
                if self.mode() != 6:
                    # Honour QGC mode changes, including Land and Return.
                    self.phase, self.reason = "idle", "Operator or failsafe changed flight mode"
                    return
                target = self.path[self.index]
                if not self.course.segment_free([p.x, p.y], target):
                    raise ValueError("Tracking drift would cut an obstacle margin; holding")
                dn, de = target[0] - p.x, target[1] - p.y
                distance = math.hypot(dn, de)
                if distance < 0.35 and math.hypot(p.vx, p.vy) < 0.35:
                    if self.index == len(self.path) - 1:
                        self.hold("Destination reached; holding")
                        return
                    self.index += 1
                else:
                    bearing = math.atan2(de, dn)
                    yaw_error = angle(bearing - yaw)
                    # Rotate at rest before moving, keeping travel inside the
                    # front of the 270-degree LiDAR rather than its rear gap.
                    yaw += max(-0.04, min(0.04, yaw_error))
                    if abs(yaw_error) < 0.25:
                        self.clear_cone(bearing)
                        speed = min(SPEED, distance * 0.6)
                        vx, vy = speed * dn / distance, speed * de / distance
                    vz = max(-0.4, min(0.4, (self.height - p.z) * 0.7))
            self.link.mav.set_position_target_local_ned_send(
                self.last_boot_ms, self.target_system, 1, M.MAV_FRAME_LOCAL_NED,
                VELOCITY_MASK, 0, 0, 0, vx, vy, vz, 0, 0, 0, yaw, 0)
        except ValueError as error:
            self.hold(str(error))

    def run(self):
        last_heartbeat = last_request = 0
        while True:
            began = time.monotonic()
            with self.lock:
                self.receive()
                if began - last_heartbeat >= 1:
                    self.link.mav.heartbeat_send(M.MAV_TYPE_ONBOARD_CONTROLLER,
                                                M.MAV_AUTOPILOT_INVALID, 0, 0,
                                                M.MAV_STATE_ACTIVE)
                    last_heartbeat = began
                if self.messages.get("HEARTBEAT") and began - last_request >= 5:
                    for message, hz in ((M.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10),
                                        (M.MAVLINK_MSG_ID_ATTITUDE, 10),
                                        (M.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10),
                                        (M.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 2)):
                        self.issue(M.MAV_CMD_SET_MESSAGE_INTERVAL, message, 1e6 / hz)
                    last_request = began
                while not self.commands.empty():
                    action, payload, result, event, deadline = self.commands.get_nowait()
                    try:
                        if time.monotonic() > deadline:
                            raise ValueError("Command expired before processing; submit again")
                        result.update(self.perform(action, payload))
                    except (ValueError, KeyError, TypeError) as error:
                        result["error"] = str(error)
                    finally:
                        event.set()
                self.control()
            time.sleep(max(0, TICK - (time.monotonic() - began)))


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    planner = Planner(Course(os.environ.get("COURSE_SDF", "/etc/px4-demo/avoidance_course.sdf")))

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send_json(self, status, value):
            body = json.dumps(value, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/api/state":
                self.send_json(200, planner.view())
            elif self.path == "/health":
                self.send_json(200, {"ok": True})
            elif self.path == "/":
                body = Path(__file__).with_name("index.html").read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_json(404, {"error": "Not found"})

        def do_POST(self):
            # Only JSON requests from this UI origin can issue commands.
            origin = self.headers.get("Origin")
            if (origin and urlsplit(origin).netloc != self.headers.get("Host")) or self.headers.get("Content-Type") != "application/json":
                self.send_json(403, {"error": "Use the planner map origin and JSON"})
                return
            action = self.path.removeprefix("/api/")
            if action not in ("preview", "start", "stop"):
                self.send_json(404, {"error": "Not found"})
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 1024:
                    raise ValueError("Invalid request length")
                payload = json.loads(self.rfile.read(size))
                if not isinstance(payload, dict):
                    raise ValueError("Expected a JSON object")
            except (ValueError, TypeError) as error:
                self.send_json(400, {"error": str(error)})
                return
            result, event = {}, threading.Event()
            planner.commands.put((action, payload, result, event, time.monotonic() + 1.5))
            if not event.wait(2):
                self.send_json(503, {"error": "Planner loop unavailable"})
                return
            self.send_json(400 if "error" in result else 200, result)

    server = ThreadingHTTPServer(("0.0.0.0", 8080), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    planner.run()


if __name__ == "__main__":
    main()
