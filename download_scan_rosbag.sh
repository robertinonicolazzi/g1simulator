#!/usr/bin/env bash
# ─── download_scan_rosbag.sh ────────────────────────────────────────────────
# Records a short rosbag inside the g1_slam Docker container on the G1 robot,
# then copies it back to this local machine for offline analysis.
#
# Captures:
#   /utlidar/cloud_livox_mid360  — raw point cloud from simulator
#   /scan                         — laser scan output from pc_to_laserscan
#   /tf                           — dynamic transforms (odom → base_link)
#   /tf_static                    — static transforms (base_link → livox_frame)
#   /odom                         — odometry
#
# Usage:
#   ./download_scan_rosbag.sh [duration_seconds]
#   Default duration: 5 seconds
#
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

# ── Configuration ─────────────────────────────────────────────────────────────
SSH_USER="rc"
SSH_HOST="35.232.79.9"
SSH_KEY="./robotic_crew_rsa_gcp"
DOCKER_CONTAINER="g1_sim_controller"

DURATION="${1:-5}"

# Paths
CONTAINER_BAG_DIR="/tmp/scan_debug_bag"
REMOTE_BAG_DIR="/home/rc/scan_debug_bag"
LOCAL_BAG_DIR="$(cd "$(dirname "$0")" && pwd)/scan_debug_bag"

TOPICS=(
    "/utlidar/cloud_livox_mid360"
    "/scan"
    "/tf"
    "/tf_static"
    "/odom"
)
TOPICS_STR="${TOPICS[*]}"

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║           /scan Debug Rosbag Download Script               ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  Remote:    ${SSH_USER}@${SSH_HOST}"
echo "║  Container: ${DOCKER_CONTAINER}"
echo "║  Duration:  ${DURATION}s"
echo "║  Topics:    $(echo "${TOPICS_STR}" | tr ' ' '\n' | head -1)"
for t in "${TOPICS[@]:1}"; do echo "║             $t"; done
echo "║  Local dir: ${LOCAL_BAG_DIR}"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

ssh_cmd() {
    ssh -o StrictHostKeyChecking=no \
        -i "${SSH_KEY}" \
        "${SSH_USER}@${SSH_HOST}" "$@"
}

scp_from() {
    scp -o StrictHostKeyChecking=no \
        -i "${SSH_KEY}" \
        -r "$1" "$2"
}

# ── Step 1: Record rosbag inside container ────────────────────────────────────
echo "▶ [1/4] Recording ${DURATION}s rosbag inside container..."
ssh_cmd "docker exec ${DOCKER_CONTAINER} bash -c '\
    source /opt/ros/humble/setup.bash && \
    [ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash; \
    rm -rf ${CONTAINER_BAG_DIR} && mkdir -p ${CONTAINER_BAG_DIR} && \
    cd ${CONTAINER_BAG_DIR} && \
    timeout $((DURATION + 2)) ros2 bag record \
        --output scan_debug \
        --max-bag-duration ${DURATION} \
        ${TOPICS_STR} \
    || true \
'"
echo "   ✔ Rosbag recorded."

# ── Step 2: Copy bag from container to remote host ────────────────────────────
echo "▶ [2/4] Copying bag from container to remote host..."
ssh_cmd "mkdir -p ${REMOTE_BAG_DIR} && \
    docker cp ${DOCKER_CONTAINER}:${CONTAINER_BAG_DIR}/. ${REMOTE_BAG_DIR}/"
echo "   ✔ Copied to remote host: ${REMOTE_BAG_DIR}"

# ── Step 3: Download bag to local machine ─────────────────────────────────────
echo "▶ [3/4] Downloading bag to local machine..."
mkdir -p "${LOCAL_BAG_DIR}"
scp_from "${SSH_USER}@${SSH_HOST}:${REMOTE_BAG_DIR}/." "${LOCAL_BAG_DIR}/"
echo "   ✔ Downloaded to: ${LOCAL_BAG_DIR}"

# ── Step 4: Cleanup remote temp files ────────────────────────────────────────
echo "▶ [4/4] Cleaning up remote temp files..."
ssh_cmd "rm -rf ${REMOTE_BAG_DIR}"
ssh_cmd "docker exec ${DOCKER_CONTAINER} rm -rf ${CONTAINER_BAG_DIR}"
echo "   ✔ Cleanup done."

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo " ✅  Rosbag downloaded successfully!"
echo ""
echo " Local files in ${LOCAL_BAG_DIR}:"
ls -lh "${LOCAL_BAG_DIR}/" 2>/dev/null || echo "   --- no files found ---"
echo ""
echo " Quick analysis commands (run locally with ROS 2):"
echo "   # Inspect bag info:"
echo "   ros2 bag info ${LOCAL_BAG_DIR}/scan_debug"
echo ""
echo "   # Check point cloud frame_id and a sample message:"
echo "   ros2 bag play ${LOCAL_BAG_DIR}/scan_debug --start-paused &"
echo "   ros2 topic echo --once /utlidar/cloud_livox_mid360 | grep frame_id"
echo ""
echo "   # Check scan values (look for inf vs real numbers):"
echo "   ros2 topic echo --once /scan | grep -E 'range|inf'"
echo ""
echo "   # Check available TF frames:"
echo "   ros2 bag play ${LOCAL_BAG_DIR}/scan_debug &"
echo "   ros2 run tf2_tools view_frames"
echo "═══════════════════════════════════════════════════════════════"
