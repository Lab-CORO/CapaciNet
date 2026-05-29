#!/bin/bash
#SBATCH --account=def-jerobg
#SBATCH --job-name=test_really_mixte_data_small_trainning_20260324_4598563
#SBATCH --output=/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/test_really_mixte_data_small_trainning_20260324_4598563/logs/slurm_%j.out
#SBATCH --error=/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/test_really_mixte_data_small_trainning_20260324_4598563/logs/slurm_%j.err
#SBATCH --time=3:00:00
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
    train3dunet --config /workspace/unet_3d/experiments/test_really_mixte_data_small_trainning_20260324_4598563/config.yaml

echo "==> Training complete: test_really_mixte_data_small_trainning_20260324_4598563"
