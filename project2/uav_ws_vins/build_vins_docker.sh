#!/bin/bash
# Build the ros:vins-fusion Docker image (VINS is compiled inside the image)

VINS_FUSION_DIR=src/VINS-Fusion-ELEC5660
DOCKER_DIR=${VINS_FUSION_DIR}/docker

# Build the Docker image from local Dockerfile
# This compiles VINS-Fusion inside the image (done once, not at runtime)
cd ${DOCKER_DIR}
sudo make build
