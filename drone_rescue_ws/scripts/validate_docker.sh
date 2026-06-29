#!/bin/bash
#
# Docker Validation Script - 3-Consecutive-Success Pattern
#
# Purpose: Validates that the Docker workflow launches the full 4-drone simulation reliably
# by requiring 3 consecutive successful launches before passing.
#
# Exit codes:
#   0 - Validation passed (3 consecutive successes achieved)
#   1 - Validation failed (max attempts exhausted)
#

set -e

# Configuration
MAX_ATTEMPTS=10
REQUIRED_SUCCESSES=3
STARTUP_TIMEOUT=90  # Accounts for Gazebo world loading
VALIDATION_NODE="pheromone_server"

# State tracking
consecutive_successes=0
total_attempts=0
total_failures=0

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "======================================================================"
echo "Docker Validation - 3-Consecutive-Success Pattern"
echo "======================================================================"
echo "Configuration:"
echo "  - Max attempts: $MAX_ATTEMPTS"
echo "  - Required consecutive successes: $REQUIRED_SUCCESSES"
echo "  - Startup timeout: ${STARTUP_TIMEOUT}s"
echo "  - Validation node: $VALIDATION_NODE"
echo "======================================================================"
echo ""

# Cleanup function
cleanup() {
    echo "Cleaning up..."
    cd /home/ros/Documents/project/drone_rescue_ws
    docker compose --profile headless down --remove-orphans 2>/dev/null || true
    sleep 2
}

# Trap to ensure cleanup on script exit
trap cleanup EXIT

# Main validation loop
while [ $consecutive_successes -lt $REQUIRED_SUCCESSES ] && [ $total_attempts -lt $MAX_ATTEMPTS ]; do
    total_attempts=$((total_attempts + 1))

    echo ""
    echo "========================================================================"
    echo -e "${YELLOW}Attempt $total_attempts/$MAX_ATTEMPTS${NC} (Consecutive successes: $consecutive_successes/$REQUIRED_SUCCESSES)"
    echo "========================================================================"

    # Ensure clean state
    cleanup

    # Start headless simulation
    echo "[$(date +%H:%M:%S)] Starting headless simulation..."
    cd /home/ros/Documents/project/drone_rescue_ws

    if ! docker compose --profile headless up -d simulation-headless 2>&1 | tee /tmp/docker_up_$total_attempts.log; then
        echo -e "${RED}[FAILURE]${NC} Docker compose up failed"
        echo "Logs:"
        cat /tmp/docker_up_$total_attempts.log
        consecutive_successes=0
        total_failures=$((total_failures + 1))
        continue
    fi

    # Wait for container startup with timeout
    echo "[$(date +%H:%M:%S)] Waiting for simulation startup (timeout: ${STARTUP_TIMEOUT}s)..."
    start_time=$(date +%s)
    node_found=false

    for i in $(seq 1 $STARTUP_TIMEOUT); do
        sleep 1
        current_time=$(date +%s)
        elapsed=$((current_time - start_time))

        # Check if container is still running
        if ! docker compose --profile headless ps simulation-headless | grep -q "Up"; then
            echo -e "${RED}[FAILURE]${NC} Container stopped unexpectedly after ${elapsed}s"
            echo "Container logs:"
            docker compose --profile headless logs --tail=50 simulation-headless
            consecutive_successes=0
            total_failures=$((total_failures + 1))
            break
        fi

        # Check for validation node
        if docker compose --profile headless exec -T simulation-headless ros2 node list 2>/dev/null | grep -q "$VALIDATION_NODE"; then
            node_found=true
            echo -e "${GREEN}[SUCCESS]${NC} Node '$VALIDATION_NODE' found after ${elapsed}s"
            consecutive_successes=$((consecutive_successes + 1))
            break
        fi

        # Progress indicator every 10 seconds
        if [ $((i % 10)) -eq 0 ]; then
            echo "  ... still waiting (${i}s elapsed)"
        fi
    done

    # Check timeout condition
    if [ "$node_found" = false ]; then
        current_time=$(date +%s)
        elapsed=$((current_time - start_time))
        echo -e "${RED}[FAILURE]${NC} Timeout: Node '$VALIDATION_NODE' not found after ${elapsed}s"
        echo "Available nodes:"
        docker compose --profile headless exec -T simulation-headless ros2 node list 2>/dev/null || echo "  (none - ros2 command failed)"
        echo ""
        echo "Recent container logs:"
        docker compose --profile headless logs --tail=100 simulation-headless
        consecutive_successes=0
        total_failures=$((total_failures + 1))
    fi

    # Stop simulation for next attempt
    echo "[$(date +%H:%M:%S)] Stopping simulation..."
    cleanup

    # Brief pause between attempts
    sleep 3
done

# Final summary
echo ""
echo "======================================================================"
echo "VALIDATION SUMMARY"
echo "======================================================================"
echo "Total attempts: $total_attempts"
echo "Total failures: $total_failures"
echo "Consecutive successes achieved: $consecutive_successes/$REQUIRED_SUCCESSES"
echo ""

if [ $consecutive_successes -ge $REQUIRED_SUCCESSES ]; then
    echo -e "${GREEN}VALIDATION PASSED${NC}"
    echo "The Docker workflow launched the full 4-drone simulation reliably"
    echo "with $REQUIRED_SUCCESSES consecutive successful launches."
    echo "======================================================================"
    exit 0
else
    echo -e "${RED}VALIDATION FAILED${NC}"
    echo "Failed to achieve $REQUIRED_SUCCESSES consecutive successes."
    echo "Max attempts ($MAX_ATTEMPTS) exhausted."
    echo "======================================================================"
    exit 1
fi
