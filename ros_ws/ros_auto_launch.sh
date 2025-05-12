#!/bin/bash

# ----------------------
# 第一部分：初始化ROS2环境
# 需根据网页1[1](@ref)和网页2[2](@ref)的ROS环境配置要求
# ----------------------
ROS_WS="/home/adam/ros_ws"
SETUP_SCRIPT="$ROS_WS/install/setup.bash"

# 验证工作空间有效性（网页3[3](@ref)提到的路径检查）
if [ ! -f "$SETUP_SCRIPT" ]; then
    echo "错误：未找到ROS工作空间环境文件 $SETUP_SCRIPT" >&2
    exit 1
fi

# ----------------------
# 第二部分：启动静态TF节点（网页11[11](@ref)的launch文件管理）
# ----------------------
gnome-terminal --title="Static TF Node" -- bash -c \
    "source $SETUP_SCRIPT && \
    ros2 launch rm_static_tf static_tf.launch.py; \
    exec bash"  # 保持终端不关闭（网页7[7](@ref)的exec技巧）

# ----------------------
# 第三部分：设备权限与串口节点（网页8[8](@ref)的权限设置）
# 使用expect处理密码输入（网页6[6](@ref)的自动化交互）
# ----------------------
gnome-terminal --title="Serial Node" -- bash -c \
    "source $SETUP_SCRIPT && \
    echo '执行设备权限修改...' && \
    expect -c 'spawn sudo chmod 666 /dev/ttyUSB0; \
        expect \"password\" {send \"1\\r\"; interact}' && \
    ros2 launch my_serial_py serial.launch.py; \
    exec bash"

# ----------------------
# 第四部分：备用终端（网页1[1](@ref)的环境持久化需求）
# ----------------------
gnome-terminal --title="ROS Environment" -- bash -c \
    "source $SETUP_SCRIPT && \
    echo 'ROS环境已加载，可执行其他命令'; \
    exec bash"

# ----------------------
# 第五部分：状态监测（网页9[9](@ref)的服务管理思路）
# ----------------------
echo -e "\n\033[32m所有终端已启动，请检查以下进程状态："
ps aux | grep -E 'static_tf|serial_py'
