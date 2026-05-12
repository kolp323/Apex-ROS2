#!/bin/bash

echo "🧠 正在检测并配置 Swap 空间..."

SWAP_FILE="/swapfile"
if [ ! -f "$SWAP_FILE" ]; then
    echo "➕ 创建 4GB Swap 文件..."
    sudo fallocate -l 4G $SWAP_FILE
    sudo chmod 600 $SWAP_FILE
    sudo mkswap $SWAP_FILE
    sudo swapon $SWAP_FILE
    echo "$SWAP_FILE none swap sw 0 0" | sudo tee -a /etc/fstab
    echo "✅ Swap 创建并启用完成"
else
    echo "✔️ Swap 文件已存在，跳过创建"
fi

echo "🔧 正在安装 ROS 相关组件和系统依赖"

# 更新系统源
sudo apt update

# 声明要安装的包
PACKAGES=(
    "ros-humble-camera-*"
    "ros-humble-image-publisher*"
    "ros-humble-test-*"
    "libuvc*"
    "ros-humble-nlohmann-json-schema-validator-vendor"
    "ros-humble-diagnostic*"
    "ros-humble-behaviortree-cpp*"
    "xtensor-*"
    "ros-humble-ompl*"
    "ros-humble-joint-state-*"
    "ros-humble-imu-*"
    "ros-humble-robot-localization*"
    "ros-humble-vision-*"
    "ros-humble-pcl-*"
    "ros-humble-compressed-image-transport"
    "ros-humble-tf-transformations"
)

# 安装所有包
for pkg in "${PACKAGES[@]}"; do
    echo "📦 安装 $pkg"
    sudo apt install -y $pkg
done

echo "✅ 所有组件安装完成。"

sudo apt install -y libgoogle-glog-dev \
                    ros-humble-bondcpp \

sudo apt install -y libgraphicsmagick++-dev \
                    libceres-dev \
                    libompl-dev 
