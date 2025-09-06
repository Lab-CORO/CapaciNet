#!/bin/bash
##
## Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
##
## NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
## property and proprietary rights in and to this material, related
## documentation and any modifications thereto. Any use, reproduction,
## disclosure or distribution of this material and related documentation
## without an express license agreement from NVIDIA CORPORATION or
## its affiliates is strictly prohibited.
##


# This script will create a dev docker. Run this script by calling `bash build_dev_docker.sh`
# If you want to build a isaac sim docker, run this script with `bash build_dev_docker.sh isaac`

# Check architecture to build:

echo "Choose your GPU card model :"
echo "1) RTX 30XX"
echo "2) RTX 40XX"
echo "3) A100"
read -p "Enter the number corresponding to your model: " choice

# Définir TORCH_CUDA_ARCH_LIST en fonction du choix de l'utilisateur
case $choice in
    1)
        image_tag="rtx30xxx"
        ;;
    2)
        image_tag="rtx40xxx"
        ;;
    3)
        image_tag="A100"
        ;;
    # 4)
    #     image_tag="x86"
    #     ;;
    *)
        echo "Choix invalide, utilisant la valeur par défaut pour RTX 30XX"
        image_tag="rtx30xxx"
        ;;
esac


if 
[ -n "$(docker images -q curobo_docker:x86)" ]
then
    echo "Docker image curobo_docker:x86 already exists"
else
    echo "Docker image curobo_docker:x86 does not exist"
    echo "Building docker image curobo_docker:x86"
    # bash ../curobo/build_docker.sh
fi

docker build  -t data_generation_docker:${image_tag} -f "x86.dockerfile" --build-arg PLATFORM=${image_tag} . 