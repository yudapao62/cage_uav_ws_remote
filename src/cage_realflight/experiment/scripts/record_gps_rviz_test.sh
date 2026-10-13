#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_record_common.sh"
[[ $# -ge 1 && $# -le 2 ]] || { echo "Usage: $0 <GPS_RVIZ_TEST_NAME> [duration_sec]" >&2; exit 2; }

test_name="$1"; duration="${2:-}"
output_dir="$RESULTS_ROOT/gps_rviz/$test_name"
mkdir -p "$output_dir/figures"
require_runtime

visualizer_config="$(cd "$EXPERIMENT_DIR/.." && pwd)/config/gps_relative_visualizer.yaml"
record_markers="$(python3 - "$visualizer_config" <<'PY'
import sys,yaml
cfg=yaml.safe_load(open(sys.argv[1],encoding='utf-8'))
print('true' if cfg.get('recording',{}).get('record_markers',True) else 'false')
PY
)"
if [[ "${RECORD_GPS_RVIZ_MARKERS:-$record_markers}" != "true" && "${RECORD_GPS_RVIZ_MARKERS:-$record_markers}" != "1" ]]; then
  markers="$(topic_value gps_test_markers)"; trails="$(topic_value gps_test_trails)"
  export EXCLUDE_TOPICS_NL="$(printf '%s\n%s\n' "$markers" "$trails")"
fi

rosparam set /gps_relative_visualizer/csv/path "$output_dir/gps_relative.csv"
cleanup_csv_param() { rosparam set /gps_relative_visualizer/csv/path "" >/dev/null 2>&1 || true; }
trap cleanup_csv_param EXIT
record_profile gps_rviz gps_rviz "$test_name" "$duration"
