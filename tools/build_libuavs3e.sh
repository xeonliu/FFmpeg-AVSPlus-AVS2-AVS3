#!/bin/sh

set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
SRC_DIR=$ROOT_DIR/third_party/uavs3e
BUILD_DIR=${UAVS3E_BUILD_DIR:-$ROOT_DIR/build/uavs3e}
PREFIX=${UAVS3E_PREFIX:-$ROOT_DIR/build/ffdeps-uavs3e}
JOBS=${JOBS:-}
ENABLE_ASM=${UAVS3E_ENABLE_ASM:-ON}

if [ -z "$JOBS" ]; then
    if command -v nproc >/dev/null 2>&1; then
        JOBS=$(nproc)
    elif command -v sysctl >/dev/null 2>&1; then
        JOBS=$(sysctl -n hw.ncpu 2>/dev/null || echo 4)
    else
        JOBS=4
    fi
fi

if [ "$(uname -s)" = "Darwin" ] && [ "$(uname -m)" = "arm64" ] && [ -z "${UAVS3E_ENABLE_ASM+x}" ]; then
    ENABLE_ASM=OFF
fi

rm -rf "$BUILD_DIR" "$PREFIX"
mkdir -p "$BUILD_DIR" "$PREFIX"

cmake -S "$SRC_DIR" -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DCOMPILE_10BIT=1 \
    -DUAVS3E_ENABLE_ASM="$ENABLE_ASM" \
    -DBUILD_SHARED_LIBS=OFF \
    ${UAVS3E_CMAKE_ARGS:-}

cmake --build "$BUILD_DIR" --target uavs3e --parallel "$JOBS"
cmake --install "$BUILD_DIR"

echo "uavs3e installed to $PREFIX"
echo "Use PKG_CONFIG_PATH=$PREFIX/lib/pkgconfig"
