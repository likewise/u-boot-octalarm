#!/bin/bash

cd $WORKSPACE
source /opt/fsl-imx-release/1.8/environment-setup-cortexa7hf-vfp-neon-poky-linux-gnueabi
make O=mx7d-phyboard-zeta mrproper
make O=mx7d-phyboard-zeta mx7d_phyboard_zeta_config
make O=mx7d-phyboard-zeta u-boot.imx
mkdir images
rm -rf images/*
cd mx7d-phyboard-zeta
tar -cvjf "${WORKSPACE}/images/u-boot-b${BUILD_ID}.tar.bz2" u-boot.imx
