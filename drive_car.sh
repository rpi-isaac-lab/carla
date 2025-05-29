#!/bin/bash
################################################################################
# This script is used to add a car and drive it via the G29 steeringwheel
#
# Author: Max Marshall
################################################################################

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cd $SCRIPT_DIR && source .venv/bin/activate && python3 "PythonAPI/max_testing/manual_control_steeringwheel.py" --res 1920x1000
