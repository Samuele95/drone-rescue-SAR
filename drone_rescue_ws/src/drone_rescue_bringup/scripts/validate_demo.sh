#!/bin/bash
# Post-demo validation script
# Checks demo completion state and recording quality
#
# Usage: validate_demo.sh [LOG_DIR]
#   LOG_DIR: Directory containing demo.log (default: latest in /tmp/drone_demo_*)
#
# Exit codes:
#   0 - All critical checks pass
#   1 - Critical check failed (no completion shot, crash errors)

set -euo pipefail

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Track overall status
VALIDATION_FAILED=false

# Logging functions
log_pass() {
    echo -e "${GREEN}[PASS]${NC} $1"
}

log_fail() {
    echo -e "${RED}[FAIL]${NC} $1"
    VALIDATION_FAILED=true
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_info() {
    echo -e "[INFO] $1"
}

echo "=== Post-Demo Validation ==="
echo ""

# ============================================================================
# DETERMINE LOG DIRECTORY
# ============================================================================

# Accept LOG_DIR argument or auto-detect latest demo log directory
LOG_DIR="${1:-}"

if [[ -z "${LOG_DIR}" ]]; then
    # Auto-detect latest demo log directory in /tmp
    LOG_DIR=$(ls -td /tmp/drone_demo_* 2>/dev/null | head -1 || echo "")
fi

if [[ -z "${LOG_DIR}" ]] || [[ ! -d "${LOG_DIR}" ]]; then
    log_fail "No demo log directory found"
    log_info "Usage: validate_demo.sh [LOG_DIR]"
    log_info "  LOG_DIR: Directory containing demo.log (or auto-detect from /tmp/drone_demo_*)"
    exit 1
fi

log_info "Using log directory: ${LOG_DIR}"

# Check for demo.log file
DEMO_LOG="${LOG_DIR}/demo.log"
if [[ ! -f "${DEMO_LOG}" ]]; then
    log_fail "demo.log not found in ${LOG_DIR}"
    exit 1
fi

log_info "Found demo.log: ${DEMO_LOG}"
echo ""

# ============================================================================
# CHECK DEMO COMPLETION STATE
# ============================================================================

log_info "Checking demo completion state..."

# Look for "Demo complete:" in demo.log
if grep -q "Demo complete:" "${DEMO_LOG}"; then
    # Extract completion information
    COMPLETION_LINE=$(grep "Demo complete:" "${DEMO_LOG}" | tail -1)
    log_pass "Demo completion marker found"
    log_info "  ${COMPLETION_LINE}"

    # Extract shot count from completion line (format: "X shots executed")
    SHOT_COUNT=$(echo "${COMPLETION_LINE}" | grep -oP '\d+(?= shots executed)' || echo "unknown")

    # Extract coverage percentage (format: "coverage XX.X%")
    COVERAGE=$(echo "${COMPLETION_LINE}" | grep -oP 'coverage \K[\d.]+' || echo "unknown")

    log_info "  Shots executed: ${SHOT_COUNT}"
    log_info "  Coverage: ${COVERAGE}%"
else
    log_fail "No demo completion marker found in logs"
fi

# Look for "completion shot" execution
if grep -q "completion shot\|coverage_75" "${DEMO_LOG}"; then
    log_pass "Completion shot executed"
else
    log_fail "Completion shot not found in logs"
fi

echo ""

# ============================================================================
# CHECK VIDEO RECORDING
# ============================================================================

log_info "Checking video recording..."

# Search for recent .mp4 files in ~/.gazebo/log/ (modified within last hour)
RECORDING_FOUND=false
VIDEO_FILE=""

for search_dir in "${HOME}/.gazebo/log" "${HOME}/.gz/log" "${LOG_DIR}"; do
    if [[ -d "${search_dir}" ]]; then
        # Find .mp4 files modified in last hour
        VIDEO_FILE=$(find "${search_dir}" -name "*.mp4" -mmin -60 -type f 2>/dev/null | head -1 || echo "")

        if [[ -n "${VIDEO_FILE}" ]]; then
            RECORDING_FOUND=true
            break
        fi
    fi
done

if [[ "${RECORDING_FOUND}" == "true" ]]; then
    # Check file size
    FILE_SIZE=$(stat -c%s "${VIDEO_FILE}" 2>/dev/null || echo "0")

    if [[ ${FILE_SIZE} -gt 0 ]]; then
        log_pass "Video recording found: ${VIDEO_FILE}"
        log_info "  File size: $((FILE_SIZE / 1024 / 1024)) MB"

        # If ffprobe available, validate video
        if command -v ffprobe &>/dev/null; then
            # Get video duration
            DURATION=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "${VIDEO_FILE}" 2>/dev/null || echo "unknown")

            # Get video resolution
            RESOLUTION=$(ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=p=0 "${VIDEO_FILE}" 2>/dev/null || echo "unknown")

            if [[ "${DURATION}" != "unknown" ]]; then
                log_info "  Duration: ${DURATION}s"
            fi

            if [[ "${RESOLUTION}" != "unknown" ]]; then
                log_info "  Resolution: ${RESOLUTION}"
            fi
        else
            log_info "  (ffprobe not available - skipping detailed video validation)"
        fi
    else
        log_warn "Video file exists but has zero size: ${VIDEO_FILE}"
    fi
else
    log_warn "No video recording found (searched ~/.gazebo/log, ~/.gz/log, ${LOG_DIR})"
    log_info "  This is non-critical if recording was disabled"
fi

echo ""

# ============================================================================
# CHECK FOR ERRORS IN LOGS
# ============================================================================

log_info "Checking for errors in logs..."

# Count ERROR lines
ERROR_COUNT=$(grep -c "ERROR" "${DEMO_LOG}" 2>/dev/null || echo "0")

# Count WARNING lines
WARNING_COUNT=$(grep -c "WARNING\|WARN" "${DEMO_LOG}" 2>/dev/null || echo "0")

if [[ ${ERROR_COUNT} -eq 0 ]]; then
    log_pass "No ERROR lines found in logs"
else
    log_warn "Found ${ERROR_COUNT} ERROR lines in logs"
    log_info "First 10 errors:"
    grep "ERROR" "${DEMO_LOG}" | head -10 | while read -r line; do
        echo "    ${line}"
    done

    # Check for critical crash errors
    if grep -q "Segmentation fault\|core dumped\|fatal error" "${DEMO_LOG}"; then
        log_fail "Critical crash error detected in logs"
    fi
fi

if [[ ${WARNING_COUNT} -gt 0 ]]; then
    log_info "Found ${WARNING_COUNT} WARNING lines in logs"
fi

echo ""

# ============================================================================
# SUMMARY
# ============================================================================

echo "=== Validation Summary ==="

if [[ "${VALIDATION_FAILED}" == "true" ]]; then
    echo -e "${RED}VALIDATION FAILED${NC}"
    echo ""
    echo "One or more critical checks failed:"
    echo "  - Demo completion marker must be present"
    echo "  - Completion shot must be executed"
    echo "  - No critical crash errors"
    echo ""
    exit 1
else
    echo -e "${GREEN}VALIDATION PASSED${NC}"
    echo ""
    echo "All critical checks passed:"
    echo "  ✓ Demo completion marker present"
    echo "  ✓ Completion shot executed"
    echo "  ✓ No critical crash errors"
    echo ""

    if [[ "${RECORDING_FOUND}" == "true" ]]; then
        echo "Video recording: ${VIDEO_FILE}"
    else
        echo "Note: No video recording found (non-critical)"
    fi
    echo ""
    exit 0
fi
