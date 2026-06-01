#!/bin/bash
#SBATCH --account=def-jerobg
#SBATCH --job-name=capa_eval
#SBATCH --output=/lustre06/project/6089348/willore/CapaciNet/unet_3d/script/eval_logs/slurm_%j.out
#SBATCH --error=/lustre06/project/6089348/willore/CapaciNet/unet_3d/script/eval_logs/slurm_%j.err
#SBATCH --time=0:05:00
#SBATCH --mem-per-cpu=10G
#SBATCH --gpus-per-node=1
#SBATCH --cpus-per-task=1
#SBATCH --export=ALL,DISABLE_DCGM=1

set -e

# Args (defaults point at the 128_002_real_data experiment):
#   $1 = config.yaml   $2 = checkpoint   $3 = val_dir   $4 = output_dir
EXP=/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/128_002_real_data_20260528_1134
CONFIG=${1:-$EXP/config.yaml}
CKPT=${2:-$EXP/checkpoint/best_checkpoint.pytorch}
VAL=${3:-$EXP/data/val}
OUT=${4:-$EXP/evaluation}
mkdir -p "$OUT" /lustre06/project/6089348/willore/CapaciNet/unet_3d/script/eval_logs
module load python/3.11.5
source /home/willore/envs/eval_env/bin/activate

python /lustre06/project/6089348/willore/CapaciNet/unet_3d/script/evaluate.py \
    --config     "$CONFIG" \
    --checkpoint "$CKPT" \
    --val_dir    "$VAL" \
    --output_dir "$OUT"

deactivate
echo "==> evaluation complete -> $OUT"
