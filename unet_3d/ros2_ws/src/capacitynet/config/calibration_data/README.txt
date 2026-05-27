INT8 Calibration Data
=====================

Place 100-500 training H5 files here before running the calibration script.

Required H5 format:
  - Dataset key : 'raw'
  - Shape       : (128, 128, 128)  [or any size — the script will resize to 128³]
  - Values      : 0 = free, 1 = occupied  (same as training data)

These are the same files produced by the CapaciNet data_generation pipeline.

To run calibration and build the INT8 engine:

    cd /home/ros2_ws/src/capacitynet
    python3 scripts/calibrate_int8.py

Build time: ~30-60 min on Jetson Orin AGX.
Output engine: config/unet3d_int8.trt

To activate the INT8 engine, update config/test_reach.yaml:
    trt_engine_path: "/home/ros2_ws/src/capacitynet/config/unet3d_int8.trt"
