#!/bin/bash
#SBATCH --account=def-jerobg
#SBATCH --job-name=train_real_data_20260310_1519
#SBATCH --output=/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/train_real_data_20260310_1519/logs/slurm_%j.out
#SBATCH --error=/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/train_real_data_20260310_1519/logs/slurm_%j.err
#SBATCH --time=10:00:00
#SBATCH --mem-per-cpu=30G
#SBATCH --gpus-per-node=1
#SBATCH --cpus-per-task=32
#SBATCH --export=ALL,DISABLE_DCGM=1

set -e

# The Apptainer bind uses a relative path, so we must cd to SCRIPT_DIR
cd /lustre06/project/6089348/willore

# ── Step 1: Format data ──────────────────────────────────────────────────────
module load python/3.11.5
source /home/willore/envs/format_data_env/bin/activate

python /lustre06/project/6089348/willore/CapaciNet/unet_3d/script/format_data.py \
    --input_dir  "/lustre06/project/6089348/willore/data_reel_valide" \
    --output_dir "/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/train_real_data_20260310_1519/data" \
    --n_copies 5 --angle_spectrum 180 --aug_seed 20 \
    --split \
    --val_ratio  0.2 \
    --seed       42

deactivate
echo "==> Data formatting complete."
echo ""

# ── Step 2: Train ─────────────────────────────────────────────────────────────
module load apptainer

apptainer exec -C --nv \
    --bind ./CapaciNet/:/workspace/ \
    /lustre06/project/6089348/willore/capa_unet.sif \
    train3dunet --config /workspace/unet_3d/experiments/train_real_data_20260310_1519/config.yaml

echo "==> Training complete: train_real_data_20260310_1519"
