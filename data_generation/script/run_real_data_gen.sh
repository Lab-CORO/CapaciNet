#!/bin/bash

# Script to run real data generation with rosbag playback
# Usage: ./run_real_data_generation.sh [rosbag_path] [sleep_time]

set -e  # Exit on error

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash

# Default values
ROSBAG_PATH="${1:-/home/ros2_ws/src/rosbag/rosbag2_2025_11_10-15_16_47/rosbag2_2025_11_10-15_16_47_0.db3}"
SLEEP_TIME="${2:-5}"  # Default sleep time in seconds

# Configuration paths
WORLD_FILE="/home/ros2_ws/src/CapaciNet/data_generation/config/leeloo_world.yaml"
CAMERAS_CONFIG="/home/ros2_ws/src/CapaciNet/data_generation/config/cameras.yaml"
BATCH_SIZE=300

# Topics to play from rosbag
TOPICS="/masked_depth_image/camera_info /masked_depth_image/image_raw"

echo "================================================"
echo "Real Data Generation Launch Script"
echo "================================================"
echo "Rosbag: $ROSBAG_PATH"
echo "Sleep time: ${SLEEP_TIME}s"
echo "World file: $WORLD_FILE"
echo "Cameras config: $CAMERAS_CONFIG"
echo "Batch size: $BATCH_SIZE"
echo "================================================"

# Check if rosbag file exists
if [ ! -f "$ROSBAG_PATH" ]; then
    echo "Error: Rosbag file not found: $ROSBAG_PATH"
    exit 1
fi

# Step 1: Play rosbag in background
echo ""
echo "[1/3] Starting rosbag playback in background..."
ROSBAG_DIR=$(dirname "$ROSBAG_PATH")
ROSBAG_FILE=$(basename "$ROSBAG_PATH")

cd "$ROSBAG_DIR"
ros2 bag play --loop "$ROSBAG_FILE" --topics $TOPICS &
ROSBAG_PID=$!

echo "Rosbag playback started (PID: $ROSBAG_PID)"

# Trap to kill rosbag on script exit
trap "echo 'Stopping rosbag playback...'; kill $ROSBAG_PID 2>/dev/null || true; exit" SIGINT SIGTERM EXIT

# Step 2: Wait for topics to be available
echo ""
echo "[2/3] Waiting ${SLEEP_TIME}s for topics to be published..."
sleep "$SLEEP_TIME"

# Verify topics are available
echo "Checking if topics are available..."
if ros2 topic list | grep -q "/masked_depth_image/camera_info"; then
    echo "✓ Topics are available"
else
    echo "⚠ Warning: Topics might not be available yet"
fi

# Step 3: Launch data generation
echo ""
echo "[3/3] Launching data generation system..."
echo "================================================"

ros2 launch data_generation generate_real_data.launch.py \
    world_file:="$WORLD_FILE" \
    cameras_config_file:="$CAMERAS_CONFIG" \
    batch_size:="$BATCH_SIZE" \
    voxel_size:=0.3  dataset_size:=5 batch_size:=300 obj_max:=00 init_batch_size:=300 use_rosbag_mode:=true scene_stabilization_max_delay_sec:=5.0

# Script will keep running until user interrupts (Ctrl+C)
# The trap will handle cleanup
