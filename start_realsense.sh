#!/bin/bash  
  
# 使用nohup启动roscore，并抑制输出到/dev/null  
nohup roscore --host 192.168.31.103 &> /dev/null &  
ROSCORE_PID=$!  
echo "roscore PID: $ROSCORE_PID" > ros_processes.pid  
  
# 等待roscore启动完成  
sleep 2
  
# 启动roslaunch，并在前台运行  
# 假设您有一个名为 my_launch_file.launch 的 launch 文件  
roslaunch realsense2_camera rs_camera.launch camera:=hs_camera align_depth:=true color_width:=640 color_height:=480 color_fps:=30 depth_width:=640 depth_height:=480 depth_fps:=30 &> /dev/null &  

ROSLAUNCH_PID=$!  
echo "roslaunch PID: $ROSLAUNCH_PID" >> ros_processes.pid  
  
# 等待roslaunch启动完成（如果需要的话）  $
sleep 2 # 这里可能需要根据实际roslaunch的启动时间来调整或删除  
  
# # 启动rqt，并在前台运行  
# rqt &  
# RQT_PID=$!  
# echo "rqt PID: $RQT_PID" >> ros_processes.pid  
  
echo "roscore with roslaunch processes started and PIDs recorded in ros_processes.pid"