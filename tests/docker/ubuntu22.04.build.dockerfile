# 使用 Ubuntu 22.04 作为基础镜像
FROM ubuntu:22.04

# 设置工作目录
WORKDIR /app

# 拷贝项目文件
COPY ./env /env

RUN apt-get update && apt-get install -y sudo

RUN chmod +x /env/ubuntu22.04.sh && /env/ubuntu22.04.sh

COPY . /app

RUN mkdir build && cd build && cmake .. && make -j

