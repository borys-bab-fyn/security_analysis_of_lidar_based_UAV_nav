#!/usr/bin/env bash
set -u
set -o pipefail

# ------------------------------------------------------------
# Batch countermeasure parameter sweep
# ------------------------------------------------------------
# Prerequisites:
#   - PX4 SITL running (make px4_sitl none) and drone armed in Altctl mode
#   - run_experiment.sh, lidar_injector_with.py, lidar_monitor.py are in same folder
#   - Python 3 & pymavlink available
# ------------------------------------------------------------

# Base arguments for run_experiment.sh (duration in seconds)
DURATION=30

# ---------------- Attack modes to test ----------------
MODES=("drift" "oscillation" "spike" "constant" "normal")

# ---------------- Countermeasure parameter sweeps ----------------
# Format: "countermeasure_name key=val key=val ..."
# Each line will be run for every MODE above

SWEEPS=(
  # slew_gate: vary max-rate-mps
  "slew_gate --max-rate-mps 0.2"
  "slew_gate --max-rate-mps 0.4"
  "slew_gate --max-rate-mps 0.6"
  "slew_gate --max-rate-mps 0.8"

  # slew_gate: vary max-jump-m
  "slew_gate --max-jump-m 0.3"
  "slew_gate --max-jump-m 0.5"
  "slew_gate --max-jump-m 0.75"
  "slew_gate --max-jump-m 1.0"

  # slew_gate: reject instead of clamp
  "slew_gate --max-rate-mps 0.4 --reject-instead-of-clamp"

  # robust_fallback: vary window-size
  "robust_fallback --window-size 3"
  "robust_fallback --window-size 5"
  "robust_fallback --window-size 7"

  # robust_fallback: vary fault-threshold
  "robust_fallback --fault-threshold 2"
  "robust_fallback --fault-threshold 3"
  "robust_fallback --fault-threshold 5"

  # robust_fallback: vary hold-seconds
  "robust_fallback --hold-seconds 0.5"
  "robust_fallback --hold-seconds 1.0"
  "robust_fallback --hold-seconds 2.0"
  "robust_fallback --hold-seconds 3.0"

  # robust_fallback: vary residual-threshold-m
  "robust_fallback --residual-threshold-m 0.3"
  "robust_fallback --residual-threshold-m 0.5"
  "robust_fallback --residual-threshold-m 0.75"
  "robust_fallback --residual-threshold-m 1.0"

  # robust_fallback: min-quality filter
  "robust_fallback --min-quality 30"
  "robust_fallback --min-quality 50"
)

# ------------------------------------------------------------
RESULTS_DIR="results_sweep_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RESULTS_DIR"

echo "Sweep started. Results will go to $RESULTS_DIR"

for mode in "${MODES[@]}"; do
  for sweep in "${SWEEPS[@]}"; do
    # Build a unique tag for this experiment
    cm_name=$(echo "$sweep" | awk '{print $1}')
    # Replace spaces and dashes with underscores for filename safety
    safe_args=$(echo "$sweep" | tr ' ' '_' | tr '-' '_')
    tag="${mode}__${safe_args}"

    echo "===== Running: mode=$mode, countermeasure=$cm_name, args=$sweep ====="

    # run_experiment.sh takes: <mode> <duration> [extra injector args...]
    # The duration is fixed; extra args include --countermeasure and the sweep params
    ./run_experiment.sh "$mode" "$DURATION" $sweep

    # run_experiment.sh saves its outputs in results/<timestamp>_<mode>/
    # Move the latest generated result directory to our sweep folder with the tag
    latest=$(ls -1dt results/2*_${mode} 2>/dev/null | head -1)
    if [[ -n "$latest" ]]; then
      mv "$latest" "${RESULTS_DIR}/${tag}"
      echo "Moved $latest to ${RESULTS_DIR}/${tag}"
    else
      echo "WARNING: No result directory found for $tag"
    fi

    # Allow a short pause for the drone to stabilise (if needed)
    sleep 2
  done
done

echo "Sweep complete. All results in $RESULTS_DIR"
