# 使用 Ubuntu 22.04 作为基础镜像
FROM ubuntu:22.04

# 设置工作目录
WORKDIR /app

# 拷贝项目文件
COPY ./env /env

# 赋予安装脚本执行权限并运行
RUN chmod +x /env/ubuntu22.04.sh && /env/ubuntu22.04.sh

COPY . /app
# 构建项目（假设你使用 CMake）
RUN mkdir build && cd build && cmake .. && make -j

