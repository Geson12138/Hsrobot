#!/bin/bash

gnome-terminal -t "roscore" -- bash -c "roscore"

# 等待一段时间以确保 roscore 已经启动
sleep 2

# 在新终端中运行的命令
CMD="roslaunch realsense2_camera rs_camera.launch align_depth:=true color_width:=640 color_height:=480 color_fps:=30 depth_width:=640 depth_height:=480 depth_fps:=30"
 
# 使用 gnome-terminal 打开新的终端，并运行命令
gnome-terminal -t "realsense_node" -- bash -c "$CMD"

# 等待一段时间以确保相机节点已经启动
sleep 2

# 在新终端中启动 rqt_image_view
gnome-terminal -t "rqt_image_view" -- bash -c "rqt"