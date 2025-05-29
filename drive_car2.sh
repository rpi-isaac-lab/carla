#!/bin/bash
################################################################################
# This script is used to add a car and drive it via the G29 steeringwheel
#
# Author: Max Marshall
################################################################################

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cd $SCRIPT_DIR && source .venv/bin/activate
./run_sim.sh &
P1=$!
sleep 10 &
P2=$!
wait $P2
python3 "PythonAPI/util/config.py" -m Town03
python3 "PythonAPI/max_testing/Katie_Logger.py" & 
python3 "PythonAPI/max_testing/manual_control_steeringwheel.py" --res 1920x1000 &


