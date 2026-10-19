FROM px4io/px4-sitl-gazebo:v1.18.0-beta2

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends xvfb x11vnc x11-utils novnc websockify \
    && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /usr/local/bin/gazebo-vnc-entrypoint
COPY gui.config /usr/local/share/gazebo-vnc/gui.config
RUN chmod 0755 /usr/local/bin/gazebo-vnc-entrypoint

EXPOSE 6080/tcp
ENTRYPOINT ["/usr/local/bin/gazebo-vnc-entrypoint"]
