#!/bin/bash  
  
# 检查 ros_processes.pid 文件是否存在  
if [ -f ros_processes.pid ]; then  
    while IFS= read -r line; do  
        PID_TYPE_PID=($line)  
        PID=${PID_TYPE_PID[2]}  
          
        # 检查进程是否存在  
        if kill -0 $PID 2>/dev/null; then  
            echo "Killing ${PID_TYPE_PID[0]} with PID: $PID"  
            # 杀死进程  
            kill $PID  
              
            # 等待进程退出（可选，如果需要确保进程已经退出）  
            # wait $PID || true  
            echo "${PID_TYPE_PID[0]} with PID $PID has been stopped."  
        else  
            echo "${PID_TYPE_PID[0]} with PID $PID is not running."  
        fi  
    done < ros_processes.pid  
      
    # 删除 PID 文件  
    rm ros_processes.pid  
else  
    echo "No PID file found for ROS processes."  
fi