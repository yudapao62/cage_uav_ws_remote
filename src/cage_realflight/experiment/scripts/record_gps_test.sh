#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_record_common.sh"
[[ $# -ge 1 && $# -le 2 ]] || { echo "Usage: $0 <GPS_TEST_NAME> [duration_sec]" >&2; exit 2; }
record_profile gps gps "$1" "${2:-}"
