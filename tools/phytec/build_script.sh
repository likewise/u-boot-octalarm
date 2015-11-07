#!/bin/bash

cd $WORKSPACE
source /opt/fsl-imx-release/1.8/environment-setup-cortexa7hf-vfp-neon-poky-linux-gnueabi
make O=mx7d-phyboard clean
make O=mx7d-phyboard mx7d_phyboard_config
make O=mx7d-phyboard u-boot.imx
mkdir images
rm -rf images/*
cd mx7d-phyboard
tar -cvjf "${WORKSPACE}/images/u-boot-b${BUILD_ID}.tar.bz2" u-boot.imx
