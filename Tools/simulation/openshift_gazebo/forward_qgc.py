#!/usr/bin/env python3
"""Keep a local QGC tunnel alive and restart it when telemetry stops."""

import argparse
import os
import shutil
import signal
import socket
import subprocess
import sys
import time


RESOURCE = "deployment/px4-gazebo-sitl"
REMOTE_PORT = 5760
PROBE_TIMEOUT = 5
TELEMETRY_TIMEOUT = 10
STARTUP_TIMEOUT = 30
RETRY_DELAY = 2


def open_monitor(port):
    stream = socket.create_connection(("127.0.0.1", port), PROBE_TIMEOUT)
    try:
        stream.settimeout(TELEMETRY_TIMEOUT)
        if not stream.recv(65536):
            raise OSError("telemetry stream closed")
        return stream
    except BaseException:
        stream.close()
        raise


def stop_forward(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=PROBE_TIMEOUT)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default="px4-loki-gazebo-demo")
    parser.add_argument("--port", default=REMOTE_PORT, type=int)
    parser.add_argument("--oc", default="oc", help="path to an oc client matching the cluster version")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535")
    oc = shutil.which(args.oc)
    if oc is None:
        parser.error("oc must be installed and available on PATH")

    # Refuse to replace another listener, and keep using the startup context
    # even if the workstation's active context changes while we are running.
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", args.port))
    context = subprocess.check_output(
        [oc, "config", "current-context"], text=True, timeout=PROBE_TIMEOUT
    ).strip()
    command = [oc, "--context", context, "-n", args.namespace]
    environment = os.environ.copy()
    environment.setdefault("KUBECTL_PORT_FORWARD_WEBSOCKETS", "true")

    while True:
        # An expired login or missing deployment needs attention, not retries.
        subprocess.run(
            command + ["get", RESOURCE, "-o", "name"],
            stdout=subprocess.DEVNULL, check=True, timeout=STARTUP_TIMEOUT,
        )
        print(f"Forwarding {context}/{args.namespace} to 127.0.0.1:{args.port}", flush=True)
        process = subprocess.Popen(
            command + ["port-forward", RESOURCE, f"{args.port}:{REMOTE_PORT}",
                       "--address=127.0.0.1"],
            env=environment, start_new_session=True,
        )
        monitor = None
        try:
            deadline = time.monotonic() + STARTUP_TIMEOUT
            while process.poll() is None and time.monotonic() < deadline:
                try:
                    monitor = open_monitor(args.port)
                    break
                except OSError:
                    time.sleep(RETRY_DELAY)
            if monitor is not None:
                print("PX4 telemetry received. Connect QGC to this TCP port. Ctrl+C stops the tunnel.", flush=True)
                # Continuously drain this read-only connection to keep the
                # transport active. Never emit synthetic GCS heartbeats.
                try:
                    while process.poll() is None and monitor.recv(65536):
                        pass
                except OSError:
                    pass
        finally:
            if monitor is not None:
                monitor.close()
            stop_forward(process)
        print("Tunnel lost or telemetry unavailable; restarting. QGC may need Connect again.", flush=True)
        time.sleep(RETRY_DELAY)


def interrupt(_signum, _frame):
    raise KeyboardInterrupt


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, interrupt)
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (OSError, subprocess.SubprocessError) as error:
        print(f"Cannot forward QGC: {error}. Check oc login, context, namespace, and port availability.", file=sys.stderr)
        sys.exit(1)
