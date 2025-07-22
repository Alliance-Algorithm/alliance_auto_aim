# Team.Alliance auto aim lib

## 安装
### 环境搭建
运行环境脚本，位于以下文件夹
```
{project_root}/env/<your system>.sh
```

### 添加进项目

由于 TBB 库的存在，你需要在库中自己解决TBB依赖问题或者简单的添加以下代码到你的CMakeLists里
```
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--copy-dt-needed-entries")
```

你可以作为一个子项目直接依赖, 或者编译安装到本机让所有人使用

如果需要安装到本机
```
git clone https://github.com/Alliance-Algorithm/alliance_auto_aim.git
cd alliance_auto_aim 
mkdir build && cd build
cmake .. && make -j
sudo make install
```

### 在项目中使用
// TODO

### ROS绑定与调试
使用Ros绑定和Ros调试库： 
[Alliance Ros auto aim](https://github.com/Alliance-Algorithm/alliance_ros_auto_aim)， 这个项目提供作为ROS包编译的方法，以及以ROS为基础的可视化调试方案，接受库中定义的接口，便于ROS开发

基础使用方式不变