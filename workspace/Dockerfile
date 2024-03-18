# 指定基础镜像
FROM ultralytics/yolov5
# 将项目文件夹添加到容器容器的指定目录（workspace）
ADD . /workspace
# 工作目录，就是程序在镜像中的所在目录
WORKDIR /workspace
# 安装依赖包,将依赖包的requirements.txt文件放到项目文件夹（workspace）
# RUN apt-get update
# RUN pip3 install --upgrade pip
RUN pip3 install -r /workspace/requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
# 容器启动后执行的命令
CMD ["python", "app.py"]
# 