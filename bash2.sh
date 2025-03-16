#!/bin/bash

# 在第一个终端中运行命令（例如启动Python脚本）
gnome-terminal -- bash -c "echo 'final_demo_arm is running'; ros2 run final_demo final_demo_arm; exec bash" &

# 在第二个终端中运行另一个命令
gnome-terminal -- bash -c "echo 'run_turtlebot is running'; ros2 run final_demo run_maze; exec bash" &

echo "all terminals are set up"