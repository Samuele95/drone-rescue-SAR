#!/bin/bash
# Automated Demo Orchestration Script
# Single-command launcher with health checks and timeout guards

set -euo pipefail

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Exit codes
EXIT_SUCCESS=0
EXIT_PREFLIGHT_FAILURE=1
EXIT_HEALTH_CHECK_FAILURE=2
EXIT_TIMEOUT=3
EXIT_RECORDING_MISSING=4

# Logging functions
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Cleanup function
cleanup() {
    local demo_pid=$1
    if [[ -n "${demo_pid}" ]] && kill -0 "${demo_pid}" 2>/dev/null; then
        log_info "Stopping demo process (PID ${demo_pid})..."
        kill -SIGTERM "${demo_pid}" 2>/dev/null || true
        sleep 5
        if kill -0 "${demo_pid}" 2>/dev/null; then
            log_warn "Process still alive, sending SIGKILL..."
            kill -SIGKILL "${demo_pid}" 2>/dev/null || true
        fi
    fi
}

# Parse demo config using grep/awk (no Python dependency)
get_config_value() {
    local key=$1
    local config_file=$2
    grep "${key}:" "${config_file}" | awk '{print $2}' | head -n1
}

log_info "=== Drone Rescue Demo Launcher ==="
log_info "Starting automated demo with health checks and timeout guards"

# ============================================================================
# PRE-FLIGHT CHECKS
# ============================================================================

log_info "Running pre-flight checks..."

# Check gz CLI available
if ! command -v gz &>/dev/null; then
    log_error "gz CLI not found. Is Gazebo Harmonic installed?"
    exit ${EXIT_PREFLIGHT_FAILURE}
fi
log_info "  [OK] gz CLI found: $(command -v gz)"

# Check ROS 2 workspace sourced
if [[ -z "${ROS_DISTRO:-}" ]]; then
    log_error "ROS_DISTRO not set. Source your ROS 2 workspace first."
    exit ${EXIT_PREFLIGHT_FAILURE}
fi
log_info "  [OK] ROS 2 ${ROS_DISTRO} environment active"

# Check drone_rescue_bringup package available
if ! ros2 pkg prefix drone_rescue_bringup &>/dev/null; then
    log_error "drone_rescue_bringup package not found. Build workspace and source install/setup.bash"
    exit ${EXIT_PREFLIGHT_FAILURE}
fi
BRINGUP_PREFIX=$(ros2 pkg prefix drone_rescue_bringup)
log_info "  [OK] drone_rescue_bringup package found at ${BRINGUP_PREFIX}"

# Load demo configuration
CONFIG_PATH="${BRINGUP_PREFIX}/share/drone_rescue_bringup/config/demo_config.yaml"
if [[ ! -f "${CONFIG_PATH}" ]]; then
    log_error "demo_config.yaml not found at ${CONFIG_PATH}"
    exit ${EXIT_PREFLIGHT_FAILURE}
fi
log_info "  [OK] Demo config loaded from ${CONFIG_PATH}"

# Parse config values
DEMO_TIMEOUT=$(get_config_value "timeout_real_seconds" "${CONFIG_PATH}")
GAZEBO_TIMEOUT=$(get_config_value "gazebo_startup_timeout" "${CONFIG_PATH}")
DIAGNOSTICS_TIMEOUT=$(get_config_value "ros_diagnostics_timeout" "${CONFIG_PATH}")

log_info "  [OK] Demo timeout: ${DEMO_TIMEOUT}s real time"

# Check FastDDS profile exists
FASTDDS_PROFILE_PATH="${BRINGUP_PREFIX}/share/drone_rescue_bringup/config/fastdds_profile.xml"
if [[ ! -f "${FASTDDS_PROFILE_PATH}" ]]; then
    log_error "fastdds_profile.xml not found at ${FASTDDS_PROFILE_PATH}"
    exit ${EXIT_PREFLIGHT_FAILURE}
fi
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_PROFILE_PATH}"
log_info "  [OK] FastDDS profile: ${FASTDDS_PROFILE_PATH}"

log_info "Pre-flight checks complete!"

# ============================================================================
# LAUNCH DEMO
# ============================================================================

log_info "Launching demo..."

# Launch demo.launch.py in background
ros2 launch drone_rescue_bringup demo.launch.py \
    num_drones:=4 \
    use_rviz:=true \
    enable_camera:=true \
    record_bag:=false &

DEMO_PID=$!
log_info "Demo launched with PID ${DEMO_PID}"

# Trap signals for cleanup
trap "cleanup ${DEMO_PID}" EXIT INT TERM

# ============================================================================
# HEALTH CHECKS
# ============================================================================

log_info "Running health checks..."

# Wait for Gazebo /clock topic
log_info "Waiting for Gazebo /clock topic (timeout ${GAZEBO_TIMEOUT}s)..."
TIMEOUT_COUNT=0
while ! ros2 topic info /clock &>/dev/null; do
    sleep 1
    TIMEOUT_COUNT=$((TIMEOUT_COUNT + 1))
    if [[ ${TIMEOUT_COUNT} -ge ${GAZEBO_TIMEOUT} ]]; then
        log_error "Gazebo /clock topic not available after ${GAZEBO_TIMEOUT}s"
        exit ${EXIT_HEALTH_CHECK_FAILURE}
    fi
done
log_info "  [OK] Gazebo /clock topic active"

# Wait for /diagnostics topic
log_info "Waiting for /diagnostics topic (timeout ${DIAGNOSTICS_TIMEOUT}s)..."
TIMEOUT_COUNT=0
while ! ros2 topic info /diagnostics &>/dev/null; do
    sleep 1
    TIMEOUT_COUNT=$((TIMEOUT_COUNT + 1))
    if [[ ${TIMEOUT_COUNT} -ge ${DIAGNOSTICS_TIMEOUT} ]]; then
        log_error "/diagnostics topic not available after ${DIAGNOSTICS_TIMEOUT}s"
        exit ${EXIT_HEALTH_CHECK_FAILURE}
    fi
done
log_info "  [OK] /diagnostics topic active"

log_info "Health checks complete! Demo is running."

# ============================================================================
# MONITOR DEMO
# ============================================================================

log_info "Monitoring demo (timeout ${DEMO_TIMEOUT}s real time)..."
log_info "Demo will run automatically until completion or timeout."

# Wait for demo with timeout
DEMO_START_TIME=$(date +%s)
while kill -0 ${DEMO_PID} 2>/dev/null; do
    sleep 5
    ELAPSED=$(($(date +%s) - DEMO_START_TIME))

    if [[ ${ELAPSED} -ge ${DEMO_TIMEOUT} ]]; then
        log_warn "Demo timeout reached (${DEMO_TIMEOUT}s), stopping demo..."
        cleanup ${DEMO_PID}
        wait ${DEMO_PID} 2>/dev/null || true
        log_info "Demo stopped after timeout"
        exit ${EXIT_TIMEOUT}
    fi

    # Log progress every 60 seconds
    if [[ $((ELAPSED % 60)) -eq 0 ]] && [[ ${ELAPSED} -gt 0 ]]; then
        log_info "Demo running... ${ELAPSED}s elapsed"
    fi
done

# Demo process exited naturally
wait ${DEMO_PID} || DEMO_EXIT_CODE=$?
DEMO_EXIT_CODE=${DEMO_EXIT_CODE:-0}

if [[ ${DEMO_EXIT_CODE} -ne 0 ]]; then
    log_error "Demo exited with error code ${DEMO_EXIT_CODE}"
    exit ${DEMO_EXIT_CODE}
fi

log_info "Demo completed successfully!"

# ============================================================================
# POST-DEMO VALIDATION
# ============================================================================

log_info "Running post-demo validation..."

# Check for video recording (if recording enabled)
RECORDING_ENABLED=$(get_config_value "enabled" "${CONFIG_PATH}" | grep -i true || echo "false")
if [[ "${RECORDING_ENABLED}" == "true" ]]; then
    log_info "Checking for video recording files..."

    # Look for .mp4 files in common recording locations
    RECORDING_FOUND=false
    for search_dir in "${HOME}/.gazebo" "${HOME}/.gz" "$(pwd)"; do
        if find "${search_dir}" -name "*.mp4" -mmin -10 2>/dev/null | grep -q .; then
            RECORDING_FOUND=true
            log_info "  [OK] Video recording found in ${search_dir}"
            break
        fi
    done

    if [[ "${RECORDING_FOUND}" == "false" ]]; then
        log_warn "No video recording found (expected .mp4 file)"
        log_warn "Check Gazebo VideoRecorder plugin configuration"
        # Don't fail on missing recording - it's a warning
    fi
fi

log_info "=== Demo Complete ==="
log_info "Demo ran successfully with all health checks passed"

exit ${EXIT_SUCCESS}
