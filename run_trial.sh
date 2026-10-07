#!/bin/bash
#
# This script is used to run the Carla standalone script built from Unreal
#
# Author: Max Marshall
# Edited by Thea Steinbach
# Runs Carla standalone, initiates Town03, requests participant number, starts logger, spawns cones, and runs blended control steering (default .5)
################################################################################
trap 'kill -INT $P4; kill -9 $P4; kill -INT $P5;  kill -INT $P1; echo The script is terminated;exit' INT
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

CARLA_BUILD_DIR="Dist/CARLA_Shipping_0.9.15-242-g715c217ad-dirty/LinuxNoEditor/"
#CARLA_BUILD_DIR="Dist/"

cd "$SCRIPT_DIR" && cd "$CARLA_BUILD_DIR" 

"/home/labstudent/carla/Dist/CARLA_Shipping_0.9.15-242-g715c217ad-dirty/LinuxNoEditor/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping" CarlaUE4 -RenderOffScreen -quality-level=Epic &
P1=$!
sleep 5 &
P2=$!
wait $P2
cd $SCRIPT_DIR && source .venv/bin/activate
python3 "/home/labstudent/carla/PythonAPI/util/config.py" -m Town03 &
P3=$!
sleep 5 &
P2a=$!
wait $P2a
echo Enter Participant Number
read v1
echo Welcome Participant $v1
python3 "/home/labstudent/carla/PythonAPI/max_testing/manual_control_steeringwheel_bike.py" --res 1920x1000 -o True -op True -w 0 -t 'EEEE' &
P5=$!
sleep 50000000 &
P2b=$!
wait $P2b
