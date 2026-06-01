#!/bin/bash
#SBATCH --account=def-jerobg
#SBATCH --job-name=eval_128_002_real_data_20260528_1134
#SBATCH --output=/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/128_002_real_data_20260528_1134/logs/slurm_eval_%j.out
#SBATCH --error=/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/128_002_real_data_20260528_1134/logs/slurm_eval_%j.err
#SBATCH --time=2:00:00
#SBATCH --mem-per-cpu=30G
#SBATCH --gpus-per-node=1
#SBATCH --cpus-per-task=2
#SBATCH --export=ALL,DISABLE_DCGM=1

set -e

module load python/3.11.5
source /home/willore/envs/eval_env/bin/activate

python /lustre06/project/6089348/willore/CapaciNet/unet_3d/script/evaluate.py \
    --config     /lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/128_002_real_data_20260528_1134/config.yaml \
    --checkpoint /lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/128_002_real_data_20260528_1134/checkpoint/best_checkpoint.pytorch \
    --output_dir /lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/128_002_real_data_20260528_1134/evaluation \
    --val_dir    /lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/128_002_real_data_20260528_1134/data/val \
    --threshold  0.5

deactivate
echo "==> Evaluation complete: 128_002_real_data_20260528_1134"
echo "    Results: /lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/128_002_real_data_20260528_1134/evaluation/"
