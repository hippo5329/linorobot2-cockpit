#!/usr/bin/env bash
# ==============================================================================
# install_deps.sh — Install the supervisor's Python prerequisites
# ==============================================================================
set -eo pipefail

echo "=================================================================="
echo "Installing Linorobot2 Cockpit Dependencies"
echo "=================================================================="

SUDO=""
if [ "$(id -u)" -ne 0 ]; then
    if command -v sudo >/dev/null 2>&1; then
        SUDO="sudo"
    else
        echo "Error: Must be run as root or with sudo installed." >&2
        exit 1
    fi
fi

if command -v apt-get >/dev/null 2>&1; then
    echo "--> Detected APT package manager (Ubuntu / Debian)"
    export DEBIAN_FRONTEND=noninteractive

    # Optional apt proxy (APT_PROXY=http://cache.example:3142) for sites with a cache.
    if [ -n "${APT_PROXY:-}" ] && [ ! -f /etc/apt/apt.conf.d/01proxy ]; then
        echo "Acquire::http::Proxy \"$APT_PROXY\";" | $SUDO tee /etc/apt/apt.conf.d/01proxy >/dev/null
    fi

    $SUDO apt-get -o APT::Update::Pre-Invoke::= update
    $SUDO apt-get install -y --no-install-recommends -o DPkg::Lock::Timeout=60 \
        python3 \
        python3-yaml \
        python3-fastapi \
        python3-uvicorn \
        python3-serial
    echo "--> APT dependencies installed successfully."
elif command -v pip3 >/dev/null 2>&1; then
    echo "--> Fallback: Installing via pip3..."
    pip3 install --break-system-packages -r "$(dirname "${BASH_SOURCE[0]}")/../requirements.txt"
else
    echo "Error: Neither apt-get nor pip3 was found." >&2
    exit 1
fi

echo "=================================================================="
echo "All dependencies ready!"
echo "=================================================================="
