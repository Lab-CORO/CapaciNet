#!/bin/bash
#SBATCH --account=def-jerobg
#SBATCH --job-name=128_002_real_data_20260528_1134
#SBATCH --output=/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/128_002_real_data_20260528_1134/logs/slurm_%j.out
#SBATCH --error=/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/128_002_real_data_20260528_1134/logs/slurm_%j.err
#SBATCH --time=30:00:00
#SBATCH --mem=500G
#SBATCH --gpus-per-node=1
#SBATCH --cpus-per-task=4
#SBATCH --export=ALL,DISABLE_DCGM=1

set -e

# The Apptainer bind uses a relative path, so we must cd to SCRIPT_DIR
cd /lustre06/project/6089348/willore
# # ── Step 1: Format data ──────────────────────────────────────────────────────
# module load python/3.11.5
# source /home/willore/envs/format_data_env/bin/activate

# python /lustre06/project/6089348/willore/CapaciNet/unet_3d/script/format_data.py \
#     --input_dir  "/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/128_002_real_data_20260528_1134/data_mixte" \
#     --output_dir "/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/128_002_real_data_20260528_1134/data" \
#      \
#     --crop 12 140 12 140 20 148 \
#     --split \
#     --val_ratio  0.2 \
#     --seed       42

# deactivate
# echo "==> Data formatting complete."
# echo ""
# ── Step 2: Train ─────────────────────────────────────────────────────────────
module load apptainer

apptainer exec -C --nv \
    --bind ./CapaciNet/:/workspace/ \
    /lustre06/project/6089348/willore/capa_unet.sif \
    train3dunet --config /workspace/unet_3d/experiments/128_002_real_data_20260528_1134/config.yaml

echo "==> Training complete: 128_002_real_data_20260528_1134"
