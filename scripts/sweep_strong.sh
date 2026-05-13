#!/usr/bin/env bash
set -u
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

DURATION=30
RESULTS_DIR="results_sweep_v2_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${RESULTS_DIR}"

# Attack modes with their STRONG parameters
# Format: "mode --extra-args"
ATTACKS=(
  "drift --drift-rate 0.5"
  "oscillation --osc-amp 5.0 --osc-period 1.0"
  "spike --spike-prob 0.3"
  "constant --constant-value 10.0"
)

# Countermeasures to test (no "none" — we already know attacks work now)
SWEEPS=(
  # slew_gate: rate limiting
  "--countermeasure slew_gate --max-rate-mps 0.2"
  "--countermeasure slew_gate --max-rate-mps 0.4"
  "--countermeasure slew_gate --max-rate-mps 0.6"
  "--countermeasure slew_gate --max-rate-mps 0.8"
  "--countermeasure slew_gate --max-rate-mps 1.0"

  # slew_gate: jump limiting
  "--countermeasure slew_gate --max-jump-m 0.5"
  "--countermeasure slew_gate --max-jump-m 1.0"
  "--countermeasure slew_gate --max-jump-m 2.0"
  "--countermeasure slew_gate --max-jump-m 3.0"

  # slew_gate: reject mode
  "--countermeasure slew_gate --max-rate-mps 0.4 --reject-instead-of-clamp"

  # robust_fallback: threshold sweep
  "--countermeasure robust_fallback --fault-threshold 1"
  "--countermeasure robust_fallback --fault-threshold 2"
  "--countermeasure robust_fallback --fault-threshold 3"
  "--countermeasure robust_fallback --fault-threshold 5"

  # robust_fallback: hold duration
  "--countermeasure robust_fallback --hold-seconds 0.5"
  "--countermeasure robust_fallback --hold-seconds 1.0"
  "--countermeasure robust_fallback --hold-seconds 2.0"
  "--countermeasure robust_fallback --hold-seconds 3.0"

  # robust_fallback: window size
  "--countermeasure robust_fallback --window-size 3"
  "--countermeasure robust_fallback --window-size 5"
  "--countermeasure robust_fallback --window-size 7"

  # robust_fallback: residual threshold
  "--countermeasure robust_fallback --residual-threshold-m 0.5"
  "--countermeasure robust_fallback --residual-threshold-m 1.0"
  "--countermeasure robust_fallback --residual-threshold-m 1.5"
  "--countermeasure robust_fallback --residual-threshold-m 2.0"
)

echo "Sweep v2 started. Results → ${RESULTS_DIR}"
echo ""

for attack in "${ATTACKS[@]}"; do
  mode=$(echo "$attack" | awk '{print $1}')
  extra_args=$(echo "$attack" | cut -d' ' -f2-)

  # First: baseline WITHOUT countermeasure for this strong attack
  echo "===== BASELINE: mode=${mode}, no countermeasure ====="
  ./run_experiment.sh "$mode" "$DURATION" --countermeasure none $extra_args
  latest=$(ls -1dt results/2*_${mode} 2>/dev/null | head -1)
  if [[ -n "$latest" ]]; then
    mv "$latest" "${RESULTS_DIR}/${mode}__no_countermeasure"
  fi
  sleep 2

  # Then: all countermeasure configurations
  for sweep in "${SWEEPS[@]}"; do
    cm_name=$(echo "$sweep" | awk '{print $2}')
    safe_tag=$(echo "${mode}__${sweep}" | tr ' ' '_' | tr '-' '_')

    echo "===== mode=${mode}, ${sweep} ====="
    ./run_experiment.sh "$mode" "$DURATION" $sweep $extra_args

    latest=$(ls -1dt results/2*_${mode} 2>/dev/null | head -1)
    if [[ -n "$latest" ]]; then
      mv "$latest" "${RESULTS_DIR}/${safe_tag}"
    fi
    sleep 2
  done
done

echo ""
echo "Sweep strong complete. All data in ${RESULTS_DIR}"
