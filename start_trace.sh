#!/bin/bash
cd ~/Desktop/program_files/vscode_files/ros2_trace_energy_mechanism/trace_energy_ws
colcon build --packages-select trace_energy
source install/setup.bash
ros2 launch trace_energy trace_start.launch.py
