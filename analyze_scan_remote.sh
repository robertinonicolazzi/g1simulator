#!/usr/bin/env bash
# ─── analyze_scan_remote.sh ──────────────────────────────────────────────────
# Records a rosbag inside the g1_slam Docker container, then plays it back and
# runs diagnostic checks — all remotely. No local ROS installation needed.
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

SSH_USER="rc"
SSH_HOST="35.232.79.9"
SSH_KEY="./robotic_crew_rsa_gcp"
DOCKER_CONTAINER="g1_sim_controller"
DURATION="${1:-5}"
CONTAINER_BAG_DIR="/tmp/scan_debug_bag"
REMOTE_TMP="/home/rc/scan_debug_tmp"

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║          Remote /scan Diagnostic — $(date '+%Y-%m-%d %H:%M')          ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  Remote:    ${SSH_USER}@${SSH_HOST}                           ║"
echo "║  Container: ${DOCKER_CONTAINER}                              ║"
echo "║  Duration:  ${DURATION}s rosbag                               ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

ssh_cmd() {
    ssh -o StrictHostKeyChecking=no -i "${SSH_KEY}" "${SSH_USER}@${SSH_HOST}" "$@"
}
scp_to() {
    scp -o StrictHostKeyChecking=no -i "${SSH_KEY}" "$1" "${SSH_USER}@${SSH_HOST}:$2"
}

# ── Write Python analysis scripts locally, then scp them ─────────────────────

cat > /tmp/pc_analyze.py << 'PYEOF'
import sys, time, math, rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PCAnalyzer(Node):
    def __init__(self):
        super().__init__('pc_analyzer')
        self.done = False
        self.sub = self.create_subscription(PointCloud2, '/utlidar/cloud_livox_mid360', self.cb, 10)

    def cb(self, msg):
        if self.done:
            return
        self.done = True
        frame = msg.header.frame_id
        pts = list(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True))
        if not pts:
            print("  frame_id :", frame)
            print("  points   : 0 (EMPTY CLOUD)")
            return
        arr = np.array(pts)
        print("  frame_id :", frame)
        print("  points   :", len(arr))
        print("  x range  : [{:.3f}, {:.3f}]".format(arr[:,0].min(), arr[:,0].max()))
        print("  y range  : [{:.3f}, {:.3f}]".format(arr[:,1].min(), arr[:,1].max()))
        print("  z range  : [{:.3f}, {:.3f}]".format(arr[:,2].min(), arr[:,2].max()))
        print("  z mean   : {:.3f}".format(arr[:,2].mean()))
        in_band = arr[(arr[:,2] >= -0.56) & (arr[:,2] <= 0.04)]
        print("  pts in z=[-0.56,0.04] (livox_frame filter band) :", len(in_band))
        # Check likely world-z band if frame is odom: livox_frame ~1.26m above ground
        # so subtract 1.26 from world z to get livox_frame z
        LIDAR_Z = 1.26
        wz = arr[:,2]
        in_world_band = arr[((wz - LIDAR_Z) >= -0.56) & ((wz - LIDAR_Z) <= 0.04)]
        print("  pts that would pass filter if transform applied (z_world in [{:.2f},{:.2f}]) : {}".format(
            LIDAR_Z - 0.56, LIDAR_Z + 0.04, len(in_world_band)))

def main():
    rclpy.init()
    n = PCAnalyzer()
    deadline = time.time() + 10
    while not n.done and time.time() < deadline:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not n.done:
        print("  ERROR: no message on /utlidar/cloud_livox_mid360 within 10s")
    n.destroy_node()
    rclpy.shutdown()

main()
PYEOF

cat > /tmp/scan_analyze.py << 'PYEOF'
import sys, time, math, rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanAnalyzer(Node):
    def __init__(self):
        super().__init__('scan_analyzer')
        self.done = False
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)

    def cb(self, msg):
        if self.done:
            return
        self.done = True
        ranges = msg.ranges
        total = len(ranges)
        infs  = sum(1 for r in ranges if math.isinf(r))
        nans  = sum(1 for r in ranges if math.isnan(r))
        valid = [r for r in ranges if not math.isinf(r) and not math.isnan(r)]
        print("  frame_id   :", msg.header.frame_id)
        print("  total beams:", total)
        print("  inf beams  : {} ({:.1f}%)".format(infs, 100*infs/total))
        print("  nan beams  :", nans)
        print("  valid beams:", len(valid))
        if valid:
            print("  range min  : {:.3f} m".format(min(valid)))
            print("  range max  : {:.3f} m".format(max(valid)))
        print("  angle_min  : {:.1f} deg".format(math.degrees(msg.angle_min)))
        print("  angle_max  : {:.1f} deg".format(math.degrees(msg.angle_max)))
        print("  range_min  : {:.3f}".format(msg.range_min))
        print("  range_max  : {:.3f}".format(msg.range_max))

def main():
    rclpy.init()
    n = ScanAnalyzer()
    deadline = time.time() + 10
    while not n.done and time.time() < deadline:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not n.done:
        print("  ERROR: no message on /scan within 10s")
    n.destroy_node()
    rclpy.shutdown()

main()
PYEOF

cat > /tmp/tf_analyze.py << 'PYEOF'
import sys, time, rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

frames_seen = set()

class TFAnalyzer(Node):
    def __init__(self):
        super().__init__('tf_analyzer')
        self.create_subscription(TFMessage, '/tf', self.cb, 100)
        self.create_subscription(TFMessage, '/tf_static', self.cb_static, 100)
        self.static_frames = []

    def cb(self, msg):
        for t in msg.transforms:
            frames_seen.add((t.header.frame_id, t.child_frame_id))

    def cb_static(self, msg):
        for t in msg.transforms:
            self.static_frames.append((t.header.frame_id, t.child_frame_id,
                t.transform.translation.x, t.transform.translation.y, t.transform.translation.z))

def main():
    rclpy.init()
    n = TFAnalyzer()
    deadline = time.time() + 6
    while time.time() < deadline:
        rclpy.spin_once(n, timeout_sec=0.1)
    print("  Dynamic TF edges:")
    for parent, child in sorted(frames_seen):
        print("    {} → {}".format(parent, child))
    print("  Static TF edges:")
    for parent, child, tx, ty, tz in n.static_frames:
        print("    {} → {}  translation=[{:.3f},{:.3f},{:.3f}]".format(parent, child, tx, ty, tz))
    n.destroy_node()
    rclpy.shutdown()

main()
PYEOF

# Upload scripts to remote host
echo "▶ Uploading analysis scripts to remote host..."
ssh_cmd "mkdir -p ${REMOTE_TMP}"
scp_to /tmp/pc_analyze.py   "${REMOTE_TMP}/pc_analyze.py"
scp_to /tmp/scan_analyze.py "${REMOTE_TMP}/scan_analyze.py"
scp_to /tmp/tf_analyze.py   "${REMOTE_TMP}/tf_analyze.py"

# Copy scripts into container
ssh_cmd "docker cp ${REMOTE_TMP}/pc_analyze.py   ${DOCKER_CONTAINER}:/tmp/pc_analyze.py"
ssh_cmd "docker cp ${REMOTE_TMP}/scan_analyze.py ${DOCKER_CONTAINER}:/tmp/scan_analyze.py"
ssh_cmd "docker cp ${REMOTE_TMP}/tf_analyze.py   ${DOCKER_CONTAINER}:/tmp/tf_analyze.py"
echo "   ✔ Scripts ready in container."
echo ""

# ── Step 1: Record ────────────────────────────────────────────────────────────
echo "▶ [1/5] Recording ${DURATION}s rosbag (live topics)..."
ssh_cmd "docker exec ${DOCKER_CONTAINER} bash -c \
    'source /opt/ros/humble/setup.bash 2>/dev/null; \
     [ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash 2>/dev/null; \
     rm -rf ${CONTAINER_BAG_DIR} && mkdir -p ${CONTAINER_BAG_DIR} && \
     cd ${CONTAINER_BAG_DIR} && \
     timeout $((DURATION + 3)) ros2 bag record \
         --output scan_debug \
         --max-bag-duration ${DURATION} \
         /utlidar/cloud_livox_mid360 /scan /tf /tf_static /odom \
         2>&1 | tail -3 || true'"
echo "   ✔ Bag recorded."
echo ""

# ── Step 2: Bag info ──────────────────────────────────────────────────────────
echo "▶ [2/5] Bag info:"
ssh_cmd "docker exec ${DOCKER_CONTAINER} bash -c \
    'source /opt/ros/humble/setup.bash 2>/dev/null; \
     ros2 bag info ${CONTAINER_BAG_DIR}/scan_debug 2>/dev/null'"
echo ""

# ── Helper: run a python analyzer while playing the bag ──────────────────────
run_with_bag() {
    local script="$1"
    ssh_cmd "docker exec ${DOCKER_CONTAINER} bash -c \
        'source /opt/ros/humble/setup.bash 2>/dev/null; \
         [ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash 2>/dev/null; \
         cd ${CONTAINER_BAG_DIR} && \
         ros2 bag play scan_debug --loop > /dev/null 2>&1 & \
         BAG_PID=\$!; \
         sleep 2; \
         python3 ${script}; \
         kill \$BAG_PID 2>/dev/null || true'"
}

# ── Step 3: Point cloud analysis ─────────────────────────────────────────────
echo "▶ [3/5] Point cloud — frame_id and z-range..."
run_with_bag /tmp/pc_analyze.py
echo ""

# ── Step 4: Scan analysis ────────────────────────────────────────────────────
echo "▶ [4/5] Laser scan — range values..."
run_with_bag /tmp/scan_analyze.py
echo ""

# ── Step 5: TF frames ────────────────────────────────────────────────────────
echo "▶ [5/5] TF frames in bag..."
run_with_bag /tmp/tf_analyze.py
echo ""

# ── Cleanup ───────────────────────────────────────────────────────────────────
ssh_cmd "docker exec ${DOCKER_CONTAINER} rm -rf ${CONTAINER_BAG_DIR} /tmp/pc_analyze.py /tmp/scan_analyze.py /tmp/tf_analyze.py 2>/dev/null || true"
ssh_cmd "rm -rf ${REMOTE_TMP}"

echo "═══════════════════════════════════════════════════════════════"
echo " ✅  Diagnostics complete."
echo " Key things to look for:"
echo "   • frame_id of cloud: 'odom' means world coords"
echo "   • z range: if frame_id=odom, world z ~ 0-3m"
echo "   • pts that pass world-z filter: should be > 0 for valid scan"
echo "   • inf beams at 100%: height filter cuts everything"
echo "   • TF edges: need odom→base_link AND base_link→livox_frame"
echo "═══════════════════════════════════════════════════════════════"
