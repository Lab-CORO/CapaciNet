#!/bin/bash
#SBATCH --account=def-jerobg
#SBATCH --job-name=gradw_bce_dice_20260531
#SBATCH --output=/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/gradw_bce_dice_20260531/logs/slurm_%j.out
#SBATCH --error=/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/gradw_bce_dice_20260531/logs/slurm_%j.err
#SBATCH --time=30:00:00
#SBATCH --mem=500G
#SBATCH --gpus-per-node=1
#SBATCH --cpus-per-task=4
#SBATCH --export=ALL,DISABLE_DCGM=1

set -e

# The Apptainer bind uses a relative path, so we must cd to SCRIPT_DIR
cd /lustre06/project/6089348/willore

# ── Train ─────────────────────────────────────────────────────────────────────
module load apptainer

apptainer exec -C --nv \
    --bind ./CapaciNet/:/workspace/ \
    --bind /lustre06/project/6089348/willore/CapaciNet/unet_3d/custom/losses.py:/opt/conda/lib/python3.10/site-packages/pytorch3dunet/unet3d/losses.py \
    /lustre06/project/6089348/willore/capa_unet.sif \
    train3dunet --config /workspace/unet_3d/experiments/gradw_bce_dice_20260531/config.yaml

echo "==> Training complete: gradw_bce_dice_20260531"
