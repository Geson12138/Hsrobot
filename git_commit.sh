#!/bin/bash  
  
# 检查是否在一个git仓库中  
if [ ! -d ".git" ]; then  
    echo "当前目录不是一个git仓库。"  
    exit 1  
fi  
  
# 获取远程仓库的最新提交  
git fetch origin BQ_Program
  
# 检查本地分支和远程分支是否有差异  
if ! git rev-list --count --left-right origin/master...HEAD | grep '^< [0-9]\+$'; then  
    echo "本地分支与远程分支同步，无需拉取更新。"  
else  
    # 提示用户是否需要拉取远程仓库的更新  
    read -p "本地分支与远程分支有差异，是否拉取远程更新？(y/n) " choice  
      
    if [[ "$choice" == "y" || "$choice" == "Y" ]]; then  
        # 执行git pull命令，这里假设拉取origin的master分支，你可以根据需要修改  
        git pull origin BQ_Program  
          
        # 检查拉取是否成功  
        if [ $? -ne 0 ]; then  
            echo "拉取失败，请检查远程仓库的连接或权限。"  
            exit 1  
        fi  
          
        echo "拉取更新成功。"  
    else  
        echo "跳过拉取操作。"  
    fi  
fi  
  
# 添加所有更改的文件到暂存区  
git add .  
  
# 提示用户输入提交信息  
read -p "请输入提交信息: " commit_message  
  
# 执行git commit命令  
git commit -m "$commit_message"  
  
# 检查上一步是否成功执行了commit  
if [ $? -ne 0 ]; then  
    echo "提交失败，请检查提交信息或仓库状态。"  
    exit 1  
fi  
  
# 提示用户是否要推送更改到远程仓库  
read -p "是否推送更改到远程仓库？(y/n) " choice  
  
# 根据用户选择执行推送  
if [[ "$choice" == "y" || "$choice" == "Y" ]]; then  
    # 执行git push命令，这里假设推送到origin的master分支，你可以根据需要修改  
    git push origin BQ_Program  
      
    # 检查推送是否成功  
    if [ $? -ne 0 ]; then  
        echo "推送失败，请检查远程仓库的连接或权限。"  
    else  
        echo "推送成功！"  
    fi  
else  
    echo "跳过推送操作。"  
fi  
  
# 输出最终消息  
echo "操作完成。"