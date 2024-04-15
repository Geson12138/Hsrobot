#!/bin/bash  
  
# 启动roslaunch，并在前台运行  
# 假设您有一个名为 my_launch_file.launch 的 launch 文件  
roslaunch realsense2_camera rs_camera.launch camera:=hs_camera align_depth:=true color_width:=640 color_height:=480 color_fps:=30 depth_width:=640 depth_height:=480 depth_fps:=30 &> /dev/null &  
ROSLAUNCH_PID=$!  
echo "roslaunch PID: $ROSLAUNCH_PID" >> ros_processes.pid   
  
echo "roslaunch processes started and PIDs recorded in ros_processes.pid"