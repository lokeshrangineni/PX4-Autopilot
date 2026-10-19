FROM alpine:3.22 AS build

RUN apk add --no-cache \
    gcc \
    g++ \
    git \
    pkgconf \
    meson \
    ninja \
    linux-headers

WORKDIR /src/mavlink-router
ARG MAVLINK_ROUTER_REF=v4
RUN git clone --depth 1 --branch "${MAVLINK_ROUTER_REF}" --recurse-submodules \
    https://github.com/mavlink-router/mavlink-router.git . \
    && meson setup -Dsystemdsystemunitdir=/usr/lib/systemd/system --buildtype=release build . \
    && ninja -C build

FROM alpine:3.22

RUN apk add --no-cache libstdc++
COPY --from=build /src/mavlink-router/build/src/mavlink-routerd /usr/local/bin/mavlink-routerd

USER 1001
EXPOSE 5760/tcp
ENTRYPOINT ["mavlink-routerd"]
