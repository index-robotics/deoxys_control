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
    libconsole-bridge-dev libtinyxml2-dev lsb-release
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

# ── pinocchio ────────────────────────────────────────────────────────────────
# libfranka >= 0.14 uses pinocchio for kinematics. Source depends on host arch:
#   amd64 — robotpkg publishes an up-to-date binary at /opt/openrobots (shared,
#           read-only install; no per-user state).
#   arm64 — robotpkg has no arm64 builds, so we use the Ubuntu `libpinocchio-dev`
#           package from the `universe` component (noble ships 2.7.0, which
#           satisfies libfranka 0.20). Installs into standard /usr paths.
DPKG_ARCH="$(dpkg --print-architecture)"
if pkg-config --exists pinocchio 2>/dev/null; then
    echo "pinocchio already discoverable via pkg-config, skipping."
elif [ "$DPKG_ARCH" = "amd64" ]; then
    CODENAME="$(lsb_release -cs)"
    echo "Installing pinocchio via robotpkg for Ubuntu ${CODENAME} (amd64)..."
    sudo mkdir -p /etc/apt/keyrings
    curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
        | sudo tee /etc/apt/keyrings/robotpkg.asc > /dev/null
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub ${CODENAME} robotpkg" \
        | sudo tee /etc/apt/sources.list.d/robotpkg.list > /dev/null
    sudo apt-get update -qq
    sudo apt-get install -y robotpkg-pinocchio
    # robotpkg installs under /opt/openrobots, not the default cmake/pkg-config
    # search paths — export so subsequent cmake invocations find it.
    export CMAKE_PREFIX_PATH="/opt/openrobots:${CMAKE_PREFIX_PATH:-}"
    export PKG_CONFIG_PATH="/opt/openrobots/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
elif [ "$DPKG_ARCH" = "arm64" ]; then
    # universe is usually already enabled on noble; be explicit in case this
    # runs on a minimal image (e.g. Jetson JetPack defaults).
    echo "Installing libpinocchio-dev from Ubuntu apt (arm64)..."
    # Evict any stale robotpkg sources file from a prior amd64-only version of
    # this script — it has no arm64 builds, so leaving it in place makes every
    # subsequent `apt-get update` noisy with 404s.
    if [ -f /etc/apt/sources.list.d/robotpkg.list ]; then
        echo "  Removing stale /etc/apt/sources.list.d/robotpkg.list (no arm64 builds)."
        sudo rm -f /etc/apt/sources.list.d/robotpkg.list
    fi
    sudo add-apt-repository -y universe >/dev/null 2>&1 || true
    sudo apt-get update -qq
    sudo apt-get install -y libpinocchio-dev
else
    echo "ERROR: unsupported architecture '${DPKG_ARCH}' — no pinocchio install path defined." >&2
    exit 1
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
if [ "$DPKG_ARCH" = "amd64" ]; then
    echo "  pinocchio  (system, /opt/openrobots/ via robotpkg)"
else
    echo "  pinocchio  (system, /usr via apt libpinocchio-dev)"
fi
