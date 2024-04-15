#!/bin/bash  
  
# 使用nohup启动roscore，并抑制输出到/dev/null  
# nohup roscore &> /dev/null &  
# ROSCORE_PID=$!  
# echo "roscore PID: $ROSCORE_PID" > ros_processes.pid  
  
# 如果您需要启动roscore，请取消上面三行的注释，并确保roscore先启动  
  
# 等待roscore启动完成（如果需要的话）  
# sleep 2  
  
# 启动roslaunch，并在后台运行  
nohup roslaunch realsense2_camera rs_camera.launch align_depth:=true color_width:=1280 color_height:=720 color_fps:=30 depth_width:=1280 depth_height:=720 depth_fps:=30 &> /dev/null &  
ROSLAUNCH_PID=$!  
echo "roslaunch PID: $ROSLAUNCH_PID" >> ros_processes.pid  
  
# 等待roslaunch启动完成（如果需要的话）  
sleep 2 # 这里可能需要根据实际roslaunch的启动时间来调整或删除  
  
# 启动rqt_image_view，并在前台运行  
rqt_image_view &  
RQT_PID=$!  
echo "rqt PID: $RQT_PID" >> ros_processes.pid  
  
echo "All ROS processes started and PIDs recorded in ros_processes.pid"
