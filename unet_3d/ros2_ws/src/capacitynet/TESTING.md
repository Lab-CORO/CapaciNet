# Testing the Gradient-Based Controller

This document describes how to test the gradient-based mobile base controller pipeline.

## Quick Start

### 1. Build the package

```bash
cd /workspace/capacitynet/ros2_ws
colcon build --packages-select capacitynet
source install/setup.bash
```

### 2. Run the mock node (basic test)

```bash
ros2 run capacitynet gradient_controller_mock
```

### 3. Run with launch file (configurable parameters)

```bash
# Basic launch
ros2 launch capacitynet gradient_controller_mock.launch.py

# With custom parameters
ros2 launch capacitynet gradient_controller_mock.launch.py \
    grid_spacing:=0.15 \
    control_frequency:=2.0 \
    gain:=1.5 \
    workspace_radius:=0.40

# With static obstacles from YAML
ros2 launch capacitynet gradient_controller_mock.launch.py \
    use_static_obstacles:=true \
    static_obstacles_yaml:=/workspace/capacitynet/config/floor_world.yml
```

## Monitor the output

### Watch velocity commands

```bash
ros2 topic echo /cmd_vel
```

### Monitor node logs

The node logs detailed information at each iteration:
- Quality scores for all 9 grid positions (3×3 layout)
- Gradient components (∂Q/∂x, ∂Q/∂y)
- Velocity command (vx, vy)
- Magnitudes

Example output:
```
[INFO] [gradient_controller_mock]:
============================================================
Iteration 1
============================================================
[INFO] [gradient_controller_mock]: Creating mock voxel map...
[INFO] [gradient_controller_mock]: Generating 9 grid transformations...
[INFO] [gradient_controller_mock]: Transformed voxels shape: torch.Size([9, 1, 100, 100, 50])
[INFO] [gradient_controller_mock]: Simulating reachability predictions...
[INFO] [gradient_controller_mock]: Predictions shape: torch.Size([9, 100, 100, 50])
[INFO] [gradient_controller_mock]: Computing control command...
[INFO] [gradient_controller_mock]:
Quality Scores (9 positions):
  Grid layout:
    0.234  0.245  0.231
    0.243  0.250  0.238
    0.229  0.242  0.225

[INFO] [gradient_controller_mock]:
Gradient:
  ∇Q = (-0.0025, +0.0080)
  |∇Q| = 0.0084

[INFO] [gradient_controller_mock]:
Velocity Command:
  v = (-0.0025, +0.0080) m/s
  |v| = 0.0084 m/s

[INFO] [gradient_controller_mock]:
Center Score: 0.250

[INFO] [gradient_controller_mock]:
Published Twist message to /cmd_vel
```

## Pipeline Components Tested

1. **ObstacleMapTransformer**
   - Creates 9 transformed voxel maps (3×3 grid)
   - Preserves static obstacles (if configured)
   - Applies transformations on GPU

2. **Simulated Reachability Prediction**
   - Mock reachability maps with Gaussian falloff from workspace center
   - Tests the expected input format for the controller

3. **GradientBasedController**
   - Computes quality score Q for each of the 9 positions
   - Calculates gradient ∇Q using central finite differences
   - Generates velocity command v = k · ∇Q
   - Applies velocity saturation

4. **WorkspaceEvaluation** (used internally)
   - Creates workspace mask (circular, 30cm radius by default)
   - Computes mean reachability within workspace

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `grid_spacing` | 0.10 | Grid spacing δ in meters |
| `control_frequency` | 1.0 | Control loop frequency (Hz) |
| `gain` | 1.0 | Proportional gain k |
| `workspace_radius` | 0.30 | Workspace radius (meters) |
| `voxel_resolution` | 0.02 | Voxel size (meters) |
| `use_static_obstacles` | false | Load static obstacles from YAML |
| `static_obstacles_yaml` | (path) | Path to YAML file with static obstacles |

## Expected Behavior

The mock node creates synthetic data where:
- Reachability is highest near the workspace center (1.0, 0.5, 0.8)
- Quality scores should show a gradient pointing toward the workspace
- Velocity commands should move the base toward better reachability

**Example interpretation:**
- If `∇Q = (+0.005, -0.003)`, the base should move in the +x, -y direction
- The velocity is proportional: `v = k · ∇Q`
- Magnitude is limited to `max_linear_vel` (0.10 m/s)

## Next Steps

After validating with the mock node:

1. Replace simulated predictions with actual UNet3D model
2. Integrate with real voxel grid service (`/unified_planner/get_voxel_grid`)
3. Subscribe to ArTag position topic for workspace center
4. Publish to actual mobile base controller

## Troubleshooting

**No GPU available:**
- The code automatically falls back to CPU if CUDA is unavailable
- Check device in logs: `Using device: cuda` or `Using device: cpu`

**Import errors:**
- Ensure the package is built: `colcon build --packages-select capacitynet`
- Source the workspace: `source install/setup.bash`

**YAML file not found:**
- Check the path in `static_obstacles_yaml` parameter
- Ensure the file exists and is readable
