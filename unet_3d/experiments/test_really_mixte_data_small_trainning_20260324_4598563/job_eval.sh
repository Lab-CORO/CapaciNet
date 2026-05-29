#!/bin/bash
#SBATCH --account=def-jerobg
#SBATCH --job-name=eval_test_really_mixte_data_small_trainning_20260324_4598563
#SBATCH --output=/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/test_really_mixte_data_small_trainning_20260324_4598563/logs/slurm_eval_%j.out
#SBATCH --error=/lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/test_really_mixte_data_small_trainning_20260324_4598563/logs/slurm_eval_%j.err
#SBATCH --time=2:00:00
#SBATCH --mem-per-cpu=30G
#SBATCH --gpus-per-node=1
#SBATCH --cpus-per-task=2
#SBATCH --export=ALL,DISABLE_DCGM=1

set -e

module load python/3.11.5
source ~/envs/eval_env/bin/activate

python /lustre06/project/6089348/willore/CapaciNet/unet_3d/script/evaluate.py \
    --config     /lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/test_really_mixte_data_small_trainning_20260324_4598563/config.yaml \
    --checkpoint /lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/test_really_mixte_data_small_trainning_20260324_4598563/checkpoint/best_checkpoint.pytorch \
    --output_dir /lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/test_really_mixte_data_small_trainning_20260324_4598563/evaluation \
    --val_dir    /lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/test_really_mixte_data_small_trainning_20260324_4598563/data/val \
    --threshold  0.5

deactivate
echo "==> Evaluation complete: test_really_mixte_data_small_trainning_20260324_4598563"
echo "    Results: /lustre06/project/6089348/willore/CapaciNet/unet_3d/experiments/test_really_mixte_data_small_trainning_20260324_4598563/evaluation/"
