#!/bin/bash
#SBATCH --account=def-jerobg
#SBATCH --job-name=l1_smooth_20260326_1622
#SBATCH --output=/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/l1_smooth_20260326_1622/logs/slurm_%j.out
#SBATCH --error=/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/l1_smooth_20260326_1622/logs/slurm_%j.err
#SBATCH --time=30:00:00
#SBATCH --mem=300G    
#SBATCH --gpus-per-node=1
#SBATCH --cpus-per-task=4
#SBATCH --export=ALL,DISABLE_DCGM=1

set -e

# The Apptainer bind uses a relative path, so we must cd to SCRIPT_DIR
cd /lustre06/project/6089348/willore

# ── Step 2: Train ─────────────────────────────────────────────────────────────
module load apptainer

apptainer exec -C --nv \
    --bind ./CapaciNet/:/workspace/ \
    /lustre06/project/6089348/willore/capa_unet.sif \
    train3dunet --config /workspace/unet_3d/experiments/BceDice_test_20260327_1642/config.yaml

echo "==> Training complete: BceDice_test_20260327_1642"
