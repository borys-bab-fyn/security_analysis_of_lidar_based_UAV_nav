#!/usr/bin/env bash
# =============================================================================
# run_experiment.sh
# -----------------------------------------------------------------------------
# Automates a full LiDAR-injection experiment cycle for the dissertation
# "Hacking Non-GPS/GPS-denied Drones: Security Analysis of LiDAR-based UAV
# Navigation".
#
# Steps performed:
#   1. Starts lidar_monitor.py in the background (listens on udp:14550).
#   2. Waits 5 s to capture baseline telemetry with *no* injection.
#   3. Runs lidar_injector.py with the selected attack mode for <DURATION>s.
#   4. Stops the monitor cleanly (SIGINT).
#   5. Runs analyze_log.py to produce plots + summary.
#   6. Copies every artefact into results/<timestamp>_<mode>/.
#
# Usage:
#   ./run_experiment.sh <mode> [duration_seconds] [extra injector args...]
#
# Examples:
#   ./run_experiment.sh normal
#   ./run_experiment.sh max_range 45
#   ./run_experiment.sh drift 60 --drift-rate 0.1
#
# Prerequisites:
#   * PX4 SITL + jMAVSim (or Gazebo) running, drone armed & hovering.
#   * QGroundControl may be running on UDP:14550 — this script also listens
#     on 14550 (input), which PX4 multicasts to, so both can coexist.
#   * Python deps installed:  pip install -r requirements.txt
# =============================================================================

set -u
set -o pipefail

# ---------- pretty-printing --------------------------------------------------
if [[ -t 1 ]]; then
    RED=$'\033[91m'; GREEN=$'\033[92m'; YELLOW=$'\033[93m'
    BLUE=$'\033[94m'; CYAN=$'\033[96m'; BOLD=$'\033[1m'; END=$'\033[0m'
else
    RED=""; GREEN=""; YELLOW=""; BLUE=""; CYAN=""; BOLD=""; END=""
fi
info()  { echo "${BLUE}[i]${END} $*"; }
ok()    { echo "${GREEN}[✓]${END} $*"; }
warn()  { echo "${YELLOW}[!]${END} $*"; }
err()   { echo "${RED}[x]${END} $*" >&2; }
banner(){ echo "${CYAN}${BOLD}$*${END}"; }

# ---------- args -------------------------------------------------------------
if [[ $# -lt 1 ]]; then
    cat <<USAGE
Usage: $(basename "$0") <mode> [duration_seconds] [extra injector args...]

Available modes: normal constant drift oscillation max_range dropout noisy spike

Examples:
  $(basename "$0") normal
  $(basename "$0") max_range 45
  $(basename "$0") drift 60 --drift-rate 0.1
USAGE
    exit 2
fi

MODE="$1"; shift
DURATION="${1:-30}"
# strip duration if it's numeric, so the rest is extra injector args
if [[ "${DURATION}" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
    shift || true
else
    DURATION=30
fi
EXTRA_ARGS=("$@")

VALID_MODES=(normal constant drift oscillation max_range dropout noisy spike)
ok_mode=0
for m in "${VALID_MODES[@]}"; do
    if [[ "$m" == "$MODE" ]]; then ok_mode=1; break; fi
done
if [[ "$ok_mode" -ne 1 ]]; then
    err "unknown mode '$MODE'. Choose one of: ${VALID_MODES[*]}"
    exit 2
fi

# ---------- paths ------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
RUN_TAG="${TIMESTAMP}_${MODE}"
RUN_DIR="${SCRIPT_DIR}/results/${RUN_TAG}"
LOG_DIR="${SCRIPT_DIR}/logs"

mkdir -p "${RUN_DIR}" "${LOG_DIR}"
MONITOR_CSV="${LOG_DIR}/monitor_${RUN_TAG}.csv"
INJECTOR_CSV="${LOG_DIR}/injector_${RUN_TAG}.csv"
MONITOR_STDOUT="${RUN_DIR}/monitor_stdout.log"
INJECTOR_STDOUT="${RUN_DIR}/injector_stdout.log"

PYTHON_BIN="${PYTHON_BIN:-python3}"

# ---------- preflight --------------------------------------------------------
banner "========================================================================"
banner "  PX4 SITL LiDAR Injection Experiment"
banner "========================================================================"
info "mode       : ${BOLD}${MODE}${END}"
info "duration   : ${DURATION} s   (baseline: 5 s + injection: ${DURATION} s)"
info "extra args : ${EXTRA_ARGS[*]:-(none)}"
info "run dir    : ${RUN_DIR}"
info "python     : ${PYTHON_BIN}"

# Basic sanity check on the scripts
for f in lidar_injector.py lidar_monitor.py analyse_log.py; do
    if [[ ! -f "${SCRIPT_DIR}/${f}" ]]; then
        err "required script missing: ${f}"
        exit 3
    fi
done

# ---------- cleanup handler --------------------------------------------------
MONITOR_PID=""
cleanup() {
    local rc=$?
    if [[ -n "${MONITOR_PID}" ]] && kill -0 "${MONITOR_PID}" 2>/dev/null; then
        warn "stopping monitor (pid=${MONITOR_PID})…"
        kill -INT "${MONITOR_PID}" 2>/dev/null || true
        # Give the monitor up to 5 s to flush its CSV.
        for _ in 1 2 3 4 5; do
            if ! kill -0 "${MONITOR_PID}" 2>/dev/null; then break; fi
            sleep 1
        done
        kill -9 "${MONITOR_PID}" 2>/dev/null || true
    fi
    exit "${rc}"
}
trap cleanup INT TERM EXIT

# ---------- start monitor ----------------------------------------------------
info "starting monitor on udpin:127.0.0.1:14550 …"
"${PYTHON_BIN}" "${SCRIPT_DIR}/lidar_monitor.py" \
    --endpoint "udpin:127.0.0.1:14550" \
    --log "${MONITOR_CSV}" \
    > "${MONITOR_STDOUT}" 2>&1 &
MONITOR_PID=$!
sleep 1
if ! kill -0 "${MONITOR_PID}" 2>/dev/null; then
    err "monitor failed to start — see ${MONITOR_STDOUT}"
    exit 4
fi
ok "monitor running (pid=${MONITOR_PID})"

# ---------- baseline window --------------------------------------------------
info "collecting 5 s of baseline telemetry (no injection)…"
sleep 5

# ---------- run injector -----------------------------------------------------
info "running injector '${MODE}' for ${DURATION} s …"
"${PYTHON_BIN}" "${SCRIPT_DIR}/lidar_injector.py" \
    --mode "${MODE}" \
    --duration "${DURATION}" \
    --log "${INJECTOR_CSV}" \
    "${EXTRA_ARGS[@]}" \
    2>&1 | tee "${INJECTOR_STDOUT}"

INJ_RC=${PIPESTATUS[0]}
if [[ "${INJ_RC}" -ne 0 ]]; then
    err "injector exited with code ${INJ_RC}"
fi

# give PX4 a moment to log the tail of the effect
sleep 2

# ---------- stop monitor -----------------------------------------------------
info "stopping monitor …"
kill -INT "${MONITOR_PID}" 2>/dev/null || true
for _ in 1 2 3 4 5 6 7 8 9 10; do
    if ! kill -0 "${MONITOR_PID}" 2>/dev/null; then break; fi
    sleep 1
done
kill -9 "${MONITOR_PID}" 2>/dev/null || true
MONITOR_PID=""
ok "monitor stopped"

# ---------- run analyser -----------------------------------------------------
info "running post-experiment analysis…"
"${PYTHON_BIN}" "${SCRIPT_DIR}/analyse_log.py" \
    --csv "${MONITOR_CSV}" \
    --outdir "${RUN_DIR}" \
    --tag "${MODE}" \
    2>&1 | tee "${RUN_DIR}/analysis_stdout.log" || \
    warn "analyser returned non-zero; continuing."

# ---------- collect artefacts -----------------------------------------------
info "collecting artefacts into ${RUN_DIR} …"
cp -f "${MONITOR_CSV}"    "${RUN_DIR}/monitor.csv"     2>/dev/null || true
cp -f "${INJECTOR_CSV}"   "${RUN_DIR}/injector.csv"    2>/dev/null || true

# Optional: snapshot the newest ULog next to the artefacts for quick reference
ULOG_DIR="${ULOG_DIR:-${HOME}/CS/final_project/PX4-Autopilot/build/px4_sitl_default/rootfs/log}"
if [[ -d "${ULOG_DIR}" ]]; then
    LATEST_ULOG="$(ls -1t "${ULOG_DIR}"/**/*.ulg 2>/dev/null | head -n 1 || true)"
    if [[ -z "${LATEST_ULOG}" ]]; then
        LATEST_ULOG="$(find "${ULOG_DIR}" -type f -name '*.ulg' -printf '%T@ %p\n' 2>/dev/null \
            | sort -nr | head -n 1 | awk '{print $2}')"
    fi
    if [[ -n "${LATEST_ULOG}" && -f "${LATEST_ULOG}" ]]; then
        cp -f "${LATEST_ULOG}" "${RUN_DIR}/px4_latest.ulg" || true
        info "copied latest ULog: $(basename "${LATEST_ULOG}")"
    else
        warn "no .ulg found under ${ULOG_DIR} — skipping ULog snapshot."
    fi
fi

# ---------- done -------------------------------------------------------------
banner "------------------------------------------------------------------------"
ok "experiment complete"
ok "artefacts : ${RUN_DIR}"
banner "========================================================================"

# Disarm the EXIT trap’s cleanup so we exit 0 cleanly.
trap - EXIT
exit 0
