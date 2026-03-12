#!/usr/bin/env bash
# fetch_deps.sh — Idempotent dependency fetcher for deoxys_control.
#
# Clones and pins all C++ dependencies needed to build deoxys. Protobuf is
# built into a local prefix (protobuf/_install/) so nothing is installed
# system-wide beyond apt packages.
#
# libfranka compatibility:
#   The default version (0.20.5) targets FR3 (Arm3Rv2) with system version
#   >= 5.9.0. Override via LIBFRANKA_VERSION for other hardware/firmware
#   combinations — see https://frankaemika.github.io for the compatibility
#   matrix.
#
# Usage:
#   ./fetch_deps.sh                            # libfranka 0.20.5 (default)
#   LIBFRANKA_VERSION=0.13.3 ./fetch_deps.sh   # override libfranka version
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEOXYS_DIR="$SCRIPT_DIR"
cd "$DEOXYS_DIR"

# ── Version pins ─────────────────────────────────────────────────────────────
LIBFRANKA_VERSION="${LIBFRANKA_VERSION:-0.20.5}"  # FR3 (Arm3Rv2), system >= 5.9.0
ZMQPP_VERSION="4.2.0"
YAML_CPP_VERSION="0.8.0"
SPDLOG_COMMIT="ac55e60488032b9acde8940a5de099541c4515da"
PROTOBUF_VERSION="v3.13.0"

# ── System packages ──────────────────────────────────────────────────────────
APT_PACKAGES=(
    build-essential cmake git libpoco-dev libeigen3-dev libzmq3-dev
    autoconf automake libtool curl make g++ unzip
    libreadline-dev bzip2 libmotif-dev libglfw3
)

MISSING=()
for pkg in "${APT_PACKAGES[@]}"; do
    if ! dpkg -s "$pkg" &>/dev/null; then
        MISSING+=("$pkg")
    fi
done

if [ ${#MISSING[@]} -gt 0 ]; then
    echo "Installing missing apt packages: ${MISSING[*]}"
    sudo apt-get update -qq
    sudo apt-get install -y "${MISSING[@]}"
else
    echo "All apt packages already installed."
fi

# ── libfranka ────────────────────────────────────────────────────────────────
if [ ! -d "libfranka" ]; then
    echo "Cloning libfranka ${LIBFRANKA_VERSION}..."
    git clone --recursive https://github.com/frankaemika/libfranka
    cd libfranka
    git checkout "$LIBFRANKA_VERSION"
    git submodule update --init --recursive
    cd ..
else
    echo "libfranka/ already exists, skipping."
fi

# Patch libfranka's SetVersionFromGit.cmake to use CMAKE_CURRENT_SOURCE_DIR
# instead of CMAKE_SOURCE_DIR, so git describe finds tags when libfranka is
# built as a subdirectory of deoxys.
SVFG="libfranka/cmake/SetVersionFromGit.cmake"
if grep -q 'CMAKE_SOURCE_DIR' "$SVFG" && ! grep -q 'CMAKE_CURRENT_SOURCE_DIR' "$SVFG"; then
    echo "Patching libfranka SetVersionFromGit.cmake..."
    sed -i 's/CMAKE_SOURCE_DIR/CMAKE_CURRENT_SOURCE_DIR/g' "$SVFG"
fi

# ── zmqpp ────────────────────────────────────────────────────────────────────
if [ ! -d "zmqpp" ]; then
    echo "Cloning zmqpp ${ZMQPP_VERSION}..."
    git clone --branch "$ZMQPP_VERSION" --depth 1 https://github.com/zeromq/zmqpp.git
else
    echo "zmqpp/ already exists, skipping."
fi

# ── yaml-cpp ─────────────────────────────────────────────────────────────────
if [ ! -d "yaml-cpp" ]; then
    echo "Cloning yaml-cpp ${YAML_CPP_VERSION}..."
    git clone --branch "$YAML_CPP_VERSION" --depth 1 https://github.com/jbeder/yaml-cpp.git
else
    echo "yaml-cpp/ already exists, skipping."
fi

# ── spdlog ───────────────────────────────────────────────────────────────────
if [ ! -d "spdlog" ]; then
    echo "Cloning spdlog (commit ${SPDLOG_COMMIT})..."
    git clone https://github.com/gabime/spdlog.git
    cd spdlog
    git checkout "$SPDLOG_COMMIT"
    cd ..
else
    echo "spdlog/ already exists, skipping."
fi

# ── protobuf (local build) ──────────────────────────────────────────────────
if [ ! -d "protobuf" ]; then
    echo "Cloning protobuf ${PROTOBUF_VERSION}..."
    git clone --recursive https://github.com/protocolbuffers/protobuf.git
    cd protobuf
    git checkout "$PROTOBUF_VERSION"
    git submodule update --init --recursive
    cd ..
fi

if [ ! -f "protobuf/_install/bin/protoc" ]; then
    echo "Building protobuf into protobuf/_install/..."
    cd protobuf
    ./autogen.sh
    ./configure --prefix="$(pwd)/_install"
    make -j"$(nproc)"
    make install
    cd ..
else
    echo "protobuf/_install/bin/protoc already exists, skipping build."
fi

echo ""
echo "All dependencies fetched. Summary:"
echo "  libfranka  ${LIBFRANKA_VERSION}"
echo "  zmqpp      ${ZMQPP_VERSION}"
echo "  yaml-cpp   ${YAML_CPP_VERSION}"
echo "  spdlog     ${SPDLOG_COMMIT:0:12}"
echo "  protobuf   ${PROTOBUF_VERSION} (local install: protobuf/_install/)"
