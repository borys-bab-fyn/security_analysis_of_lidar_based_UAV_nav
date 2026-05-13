#!/usr/bin/env bash
set -u
set -o pipefail

# =============================================================================
# sweep_countermeasures.sh
# -----------------------------------------------------------------------------
# Automated parameter sweep for LiDAR injection countermeasures.
#
# Prerequisites:
#   - PX4 SITL running (make px4_sitl none) and drone hovering at 2.5 m
#     in Altitude mode (commander arm -f && commander takeoff)
#   - run_experiment.sh, lidar_injector.py, lidar_monitor.py in same folder
#   - Python 3 + pymavlink installed
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

DURATION=30   # seconds of injection per experiment
RESULTS_DIR="results_sweep_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${RESULTS_DIR}"

# Color helpers (optional)
if [[ -t 1 ]]; then
    GREEN=$'\033[92m'; YELLOW=$'\033[93m'; CYAN=$'\033[96m'; END=$'\033[0m'
else
    GREEN=""; YELLOW=""; CYAN=""; END=""
fi

# ------------------------- Attack modes to test -------------------------
MODES=("normal" "constant" "drift" "oscillation" "spike")

# ------------------------- Countermeasure sweeps -------------------------
# Format: "--countermeasure <name> [extra params ...]"
# The injector will parse them correctly.
SWEEPS=(
  # ---- slew_gate: vary max-rate-mps ----
  "--countermeasure slew_gate --max-rate-mps 0.2"
  "--countermeasure slew_gate --max-rate-mps 0.4"
  "--countermeasure slew_gate --max-rate-mps 0.6"
  "--countermeasure slew_gate --max-rate-mps 0.8"

  # ---- slew_gate: vary max-jump-m ----
  "--countermeasure slew_gate --max-jump-m 0.3"
  "--countermeasure slew_gate --max-jump-m 0.5"
  "--countermeasure slew_gate --max-jump-m 0.75"
  "--countermeasure slew_gate --max-jump-m 1.0"

  # ---- slew_gate: reject instead of clamp ----
  "--countermeasure slew_gate --max-rate-mps 0.4 --reject-instead-of-clamp"

  # ---- robust_fallback: vary window-size ----
  "--countermeasure robust_fallback --window-size 3"
  "--countermeasure robust_fallback --window-size 5"
  "--countermeasure robust_fallback --window-size 7"

  # ---- robust_fallback: vary fault-threshold ----
  "--countermeasure robust_fallback --fault-threshold 2"
  "--countermeasure robust_fallback --fault-threshold 3"
  "--countermeasure robust_fallback --fault-threshold 5"

  # ---- robust_fallback: vary hold-seconds ----
  "--countermeasure robust_fallback --hold-seconds 0.5"
  "--countermeasure robust_fallback --hold-seconds 1.0"
  "--countermeasure robust_fallback --hold-seconds 2.0"
  "--countermeasure robust_fallback --hold-seconds 3.0"

  # ---- robust_fallback: vary residual-threshold-m ----
  "--countermeasure robust_fallback --residual-threshold-m 0.3"
  "--countermeasure robust_fallback --residual-threshold-m 0.5"
  "--countermeasure robust_fallback --residual-threshold-m 0.75"
  "--countermeasure robust_fallback --residual-threshold-m 1.0"

  # ---- robust_fallback: min-quality ----
  "--countermeasure robust_fallback --min-quality 30"
  "--countermeasure robust_fallback --min-quality 50"
)

echo "${CYAN}Sweep started.${END} Results → ${RESULTS_DIR}"
echo ""

# -------------------------------------------------------------------------
# Helper: quick check that the drone is armed and in a flying mode.
# We use lidar_monitor to peek at the heartbeat (mode flag) but a simple
# MAVLink connection check can be enough.  Here we rely on run_experiment.sh
# to work correctly; if the drone disarms the monitor log will show it.
# -------------------------------------------------------------------------

for mode in "${MODES[@]}"; do

  # ---- baseline (no countermeasure) for this mode ----
  if [[ "$mode" != "normal" ]]; then   # normal mode already is baseline-ish
    echo "===== ${GREEN}Baseline: mode=${mode}, no countermeasure${END} ====="
    ./run_experiment.sh "$mode" "$DURATION" --countermeasure none
    latest=$(ls -1dt results/2*_${mode} 2>/dev/null | head -1)
    if [[ -n "$latest" ]]; then
      tag="${mode}__no_countermeasure"
      mv "$latest" "${RESULTS_DIR}/${tag}"
      echo "Moved ${latest} → ${RESULTS_DIR}/${tag}"
    fi
    sleep 2
  fi

  # ---- sweeps for this mode ----
  for sweep in "${SWEEPS[@]}"; do
    # Extract a readable tag
    cm_name=$(echo "$sweep" | awk '{print $2}')   # after --countermeasure
    safe_args=$(echo "$sweep" | tr ' ' '_' | tr '-' '_')
    tag="${mode}__${safe_args}"

    echo ""
    echo "===== ${YELLOW}mode=${mode}, sweep=${sweep}${END} ====="

    ./run_experiment.sh "$mode" "$DURATION" $sweep

    latest=$(ls -1dt results/2*_${mode} 2>/dev/null | head -1)
    if [[ -n "$latest" ]]; then
      mv "$latest" "${RESULTS_DIR}/${tag}"
      echo "Moved ${latest} → ${RESULTS_DIR}/${tag}"
    else
      echo "WARNING: no result directory for $tag"
    fi
    sleep 2
  done
done

echo ""
echo "${CYAN}Sweep complete.${END} All data in ${RESULTS_DIR}"
