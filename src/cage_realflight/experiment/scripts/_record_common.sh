#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXPERIMENT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_FILE="${EXPERIMENT_CONFIG:-$EXPERIMENT_DIR/config/experiment_topics.yaml}"
RESULTS_ROOT="${EXPERIMENT_RESULTS_ROOT:-$EXPERIMENT_DIR/results}"

require_runtime() {
  command -v rostopic >/dev/null || { echo "rostopic not found; source ROS and cage_uav_ws/devel/setup.bash" >&2; exit 2; }
  command -v rosbag >/dev/null || { echo "rosbag not found; source ROS first" >&2; exit 2; }
  rostopic list >/dev/null 2>&1 || { echo "ROS master is unavailable" >&2; exit 2; }
  [[ -f "$CONFIG_FILE" ]] || { echo "Missing config: $CONFIG_FILE" >&2; exit 2; }
}

profile_topics() {
  local profile="$1"
  python3 - "$CONFIG_FILE" "$profile" <<'PY'
import sys, yaml
cfg=yaml.safe_load(open(sys.argv[1], encoding='utf-8'))
for key in cfg['profiles'].get(sys.argv[2], []):
    value=cfg['topics'].get(key)
    if value:
        print(value)
PY
}

topic_value() {
  python3 - "$CONFIG_FILE" "$1" <<'PY'
import sys, yaml
cfg=yaml.safe_load(open(sys.argv[1], encoding='utf-8'))
print(cfg.get('topics', {}).get(sys.argv[2], '') or '')
PY
}

write_metadata() {
  local output_dir="$1" test_name="$2" experiment_type="$3" start_iso="$4" end_iso="$5" duration="$6"
  shift 6
  TEST_NAME="$test_name" EXPERIMENT_TYPE="$experiment_type" START_ISO="$start_iso" END_ISO="$end_iso" \
  DURATION_SEC="$duration" OUTPUT_DIR="$output_dir" CONFIG_FILE="$CONFIG_FILE" TOPICS_NL="$(printf '%s\n' "$@")" \
  ANTENNA_DX_M="${ANTENNA_DX_M:-0.0}" ANTENNA_DY_M="${ANTENNA_DY_M:-0.0}" ANTENNA_DZ_M="${ANTENNA_DZ_M:-0.0}" \
  python3 - <<'PY'
import os, yaml
data={
  'test_name': os.environ['TEST_NAME'],
  'experiment_type': os.environ['EXPERIMENT_TYPE'],
  'start_time': os.environ['START_ISO'],
  'end_time': os.environ['END_ISO'] or None,
  'duration_sec': None if os.environ['DURATION_SEC']=='pending' else float(os.environ['DURATION_SEC']),
  'time_basis': 'rosbag receipt time for offline alignment',
  'topics': [v for v in os.environ['TOPICS_NL'].splitlines() if v],
  'config_file': os.environ['CONFIG_FILE'],
}
if data['experiment_type']=='gps':
  cfg=yaml.safe_load(open(os.environ['CONFIG_FILE'], encoding='utf-8'))
  data.update({
    'ground_gps_topic': cfg['topics']['ground_gps'],
    'uav_gps_topic': cfg['topics']['uav_gps'],
    'ground_anchor_id': os.environ.get('GROUND_ANCHOR_ID', cfg['gps_analysis']['ground_anchor_id']),
    'antenna_dx': float(os.environ['ANTENNA_DX_M']),
    'antenna_dy': float(os.environ['ANTENNA_DY_M']),
    'antenna_dz': float(os.environ['ANTENNA_DZ_M']),
    'antenna_offset_definition': 'Ground relative to UAV in East/North/Up metres',
  })
with open(os.path.join(os.environ['OUTPUT_DIR'],'metadata.yaml'),'w',encoding='utf-8') as f:
  yaml.safe_dump(data,f,allow_unicode=True,sort_keys=False)
PY
}

record_profile() {
  local experiment_type="$1" profile="$2" test_name="$3" duration="${4:-}"
  [[ "$test_name" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "Invalid test name" >&2; exit 2; }
  if [[ -n "$duration" && ! "$duration" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "Duration must be seconds" >&2; exit 2
  fi
  require_runtime
  mapfile -t required < <(profile_topics "${profile}_required")
  mapfile -t optional < <(profile_topics "${profile}_optional")
  mapfile -t live < <(rostopic list)
  local missing=() selected=() topic
  for topic in "${required[@]}"; do
    if printf '%s\n' "${live[@]}" | grep -Fxq "$topic"; then selected+=("$topic"); else missing+=("$topic"); fi
  done
  if ((${#missing[@]})); then
    printf 'Required topic not found: %s\n' "${missing[@]}" >&2
    exit 3
  fi
  for topic in "${optional[@]}"; do
    if [[ -n "${EXCLUDE_TOPICS_NL:-}" ]] && printf '%s\n' "$EXCLUDE_TOPICS_NL" | grep -Fxq "$topic"; then
      echo "Excluded by recording config: $topic" >&2
      continue
    fi
    if printf '%s\n' "${live[@]}" | grep -Fxq "$topic"; then selected+=("$topic"); else echo "Optional topic not found: $topic" >&2; fi
  done
  local output_dir="$RESULTS_ROOT/$experiment_type/$test_name"
  [[ ! -e "$output_dir/raw.bag" && ! -e "$output_dir/raw.bag.active" ]] || { echo "Raw bag already exists: $output_dir" >&2; exit 4; }
  mkdir -p "$output_dir/figures"
  [[ -f "$output_dir/notes.txt" ]] || cat > "$output_dir/notes.txt" <<'EOF'
Location / weather:
Equipment / firmware:
Antenna placement:
Operator actions and exact event times:
Abnormal observations:
Manual response confirmed (override experiment): YES / NO / NOT TESTED
EOF
  local start_iso start_epoch end_iso end_epoch elapsed rc=0
  start_iso="$(date --iso-8601=seconds)"; start_epoch="$(date +%s.%N)"
  write_metadata "$output_dir" "$test_name" "$experiment_type" "$start_iso" "" pending "${selected[@]}"
  echo "Recording ${#selected[@]} topics to $output_dir/raw.bag"
  printf '  %s\n' "${selected[@]}"
  if [[ -n "$duration" ]]; then
    timeout --signal=INT --kill-after=15 "${duration}s" rosbag record --lz4 -O "$output_dir/raw.bag" "${selected[@]}" || rc=$?
    [[ "$rc" == 0 || "$rc" == 124 || "$rc" == 130 ]] || exit "$rc"
  else
    echo "Press Ctrl+C once to stop and finalize the bag."
    rosbag record --lz4 -O "$output_dir/raw.bag" "${selected[@]}" || rc=$?
    [[ "$rc" == 0 || "$rc" == 130 ]] || exit "$rc"
  fi
  end_iso="$(date --iso-8601=seconds)"; end_epoch="$(date +%s.%N)"
  elapsed="$(python3 -c "print(max(0.0,float('$end_epoch')-float('$start_epoch')))")"
  write_metadata "$output_dir" "$test_name" "$experiment_type" "$start_iso" "$end_iso" "$elapsed" "${selected[@]}"
  [[ -f "$output_dir/raw.bag" ]] || { echo "rosbag did not finalize raw.bag" >&2; exit 5; }
  rosbag info --yaml "$output_dir/raw.bag" > "$output_dir/bag_info.yaml"
  echo "Saved: $output_dir"
}
