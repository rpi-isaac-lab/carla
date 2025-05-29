#!/bin/bash
#
# This script is used to run the Carla standalone script built from Unreal
#
# Author: Max Marshall
################################################################################

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

CARLA_BUILD_DIR="Dist/CARLA_Shipping_0.9.15-242-g715c217ad-dirty/LinuxNoEditor/"
#CARLA_BUILD_DIR="Dist/"

cd "$SCRIPT_DIR" && cd "$CARLA_BUILD_DIR" && ./CarlaUE4.sh -RenderOffScreen -quality-level=Epic
