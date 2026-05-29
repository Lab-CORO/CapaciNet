#!/bin/bash
#SBATCH --account=def-jerobg
#SBATCH --job-name=test_real_data_small_trainning_20260319_1451
#SBATCH --output=/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/test_real_data_small_trainning_20260319_1451/logs/slurm_%j.out
#SBATCH --error=/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/test_real_data_small_trainning_20260319_1451/logs/slurm_%j.err
#SBATCH --time=3:00:00
#SBATCH --mem-per-cpu=30G
#SBATCH --gpus-per-node=1
#SBATCH --cpus-per-task=32
#SBATCH --export=ALL,DISABLE_DCGM=1

set -e

# The Apptainer bind uses a relative path, so we must cd to SCRIPT_DIR
cd /lustre06/project/6089348/willore

# ── Step 2: Train ─────────────────────────────────────────────────────────────
module load apptainer

apptainer exec -C --nv \
    --bind ./CapaciNet/:/workspace/ \
    /lustre06/project/6089348/willore/capa_unet.sif \
    train3dunet --config /workspace/unet_3d/experiments/test_real_data_small_trainning_20260319_1451/config.yaml

echo "==> Training complete: test_real_data_small_trainning_20260319_1451"
