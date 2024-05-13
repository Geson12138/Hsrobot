#!/bin/bash  
  
# 查找与 RealSense 相关的所有进程并获取 PID  
realsense_pids=$(ps aux | grep realsense | grep -v grep | awk '{print $2}')  
  
# 检查是否找到了进程  
if [[ -z "$realsense_pids" ]]; then  
    echo "没有找到与 RealSense 相关的进程"  
else  
    # 遍历所有 PID 并杀死它们  
    for pid in $realsense_pids; do  
        echo "正在杀死 PID: $pid"  
        kill "$pid"  
        # 你可以添加 -9 选项来强制杀死进程，但这通常是不必要的  
        # kill -9 "$pid"  
    done  
      
    echo "所有与 RealSense 相关的进程已被杀死"  
fi