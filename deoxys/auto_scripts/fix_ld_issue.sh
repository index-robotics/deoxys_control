#!/bin/bash

# Get the absolute path to the deoxys directory (parent of auto_scripts)
# Use BASH_SOURCE if available (when sourced), otherwise use $0
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-${0}}")" && pwd)"
DEOXYS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
export LD_LIBRARY_PATH="${DEOXYS_DIR}/lib:${LD_LIBRARY_PATH}"
