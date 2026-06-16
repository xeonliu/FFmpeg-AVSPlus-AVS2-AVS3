#!/bin/sh

set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
PREFIX=${UAVS3E_PREFIX:-$ROOT_DIR/build/ffdeps-uavs3e}
FFMPEG_BUILD_DIR=${FFMPEG_BUILD_DIR:-$ROOT_DIR}
UAVS3D_DIR=${UAVS3D_DIR:-/tmp/uavs3d-ffmpeg-avs3-verify}
UAVS3D_COMMIT=${UAVS3D_COMMIT:-0e20d2c291853f196c68922a264bcd8471d75b68}
JOBS=${JOBS:-}

if [ -z "$JOBS" ]; then
    if command -v nproc >/dev/null 2>&1; then
        JOBS=$(nproc)
    elif command -v sysctl >/dev/null 2>&1; then
        JOBS=$(sysctl -n hw.ncpu 2>/dev/null || echo 4)
    else
        JOBS=4
    fi
fi

"$ROOT_DIR/tools/build_libuavs3e.sh"

mkdir -p "$FFMPEG_BUILD_DIR"

if [ -f "$ROOT_DIR/config.mak" ]; then
    make -C "$ROOT_DIR" distclean
elif [ "$FFMPEG_BUILD_DIR" != "$ROOT_DIR" ]; then
    rm -f \
        "$ROOT_DIR/config.asm" \
        "$ROOT_DIR/config.h" \
        "$ROOT_DIR/config_components.h" \
        "$ROOT_DIR/config.mak"
fi

cd "$FFMPEG_BUILD_DIR"
PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig" \
    "$ROOT_DIR/configure" \
        --disable-doc \
        --disable-autodetect \
        --enable-gpl \
        --enable-libuavs3e \
        --enable-encoder=libuavs3e \
        --pkg-config-flags=--static

make -j"$JOBS"

"$FFMPEG_BUILD_DIR/ffmpeg" -hide_banner -encoders | grep libuavs3e

if [ ! -d "$UAVS3D_DIR/.git" ]; then
    rm -rf "$UAVS3D_DIR"
    git clone https://github.com/uavs3/uavs3d.git "$UAVS3D_DIR"
fi

git -C "$UAVS3D_DIR" fetch --depth 1 origin "$UAVS3D_COMMIT"
git -C "$UAVS3D_DIR" checkout --detach "$UAVS3D_COMMIT"

rm -rf "$UAVS3D_DIR/build/verify"
cmake -S "$UAVS3D_DIR" -B "$UAVS3D_DIR/build/verify" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DCOMPILE_10BIT=1 \
    -DBUILD_SHARED_LIBS=OFF
cmake --build "$UAVS3D_DIR/build/verify" --parallel "$JOBS"

UAVS3D_BIN=$UAVS3D_DIR/build/verify/uavs3dec
if [ ! -x "$UAVS3D_BIN" ]; then
    UAVS3D_BIN=$UAVS3D_DIR/build/verify/uavs3d
fi

OUT_DIR=${OUT_DIR:-/tmp/ffmpeg-uavs3e-roundtrip}
rm -rf "$OUT_DIR"
mkdir -p "$OUT_DIR"

"$FFMPEG_BUILD_DIR/ffmpeg" -hide_banner -y \
    -f lavfi -i testsrc2=size=128x72:rate=25 \
    -frames:v 10 \
    -pix_fmt yuv420p10le \
    -c:v libuavs3e \
    "$OUT_DIR/avs3-test.avs3"

test -s "$OUT_DIR/avs3-test.avs3"

"$UAVS3D_BIN" -i "$OUT_DIR/avs3-test.avs3" -o "$OUT_DIR/avs3-test.yuv"
test -s "$OUT_DIR/avs3-test.yuv"

echo "round trip ok: $OUT_DIR/avs3-test.avs3 -> $OUT_DIR/avs3-test.yuv"
