#!/usr/bin/env bash
# install.bash - 安装 tag_nav 项目依赖，并可选编译
# 固定工作区为脚本所在目录的父目录

set -e

# 颜色
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}[INFO]${NC} 开始安装 tag_nav 依赖..."

# 确定工作区根目录（脚本的父目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(dirname "$SCRIPT_DIR")"
echo -e "${BLUE}[INFO]${NC} 工作区路径: $WORKSPACE"

# 检查 ROS 环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}[WARNING]${NC} 未检测到 ROS 环境，请先 source /opt/ros/noetic/setup.bash"
    exit 1
fi
echo -e "${BLUE}[INFO]${NC} ROS 发行版: $ROS_DISTRO"

# 1. 更新系统包列表
echo -e "${BLUE}[INFO]${NC} 更新系统包列表..."
sudo apt update -qq

# 2. 安装系统依赖
echo -e "${BLUE}[INFO]${NC} 安装系统依赖 (build-essential, cmake, git, etc.)..."
sudo apt install -y \
    build-essential cmake git wget curl \
    python3-pip python3-catkin-tools \
    libopencv-dev libjsoncpp-dev libapriltag-dev apriltag \
    libyaml-cpp-dev libeigen3-dev libboost-all-dev \
    libtinyxml2-dev liburdfdom-dev libsdformat-dev \
    libgazebo11-dev

# 3. 安装 ROS 相关包
echo -e "${BLUE}[INFO]${NC} 安装 ROS 依赖包..."
sudo apt install -y \
    ros-${ROS_DISTRO}-desktop-full \
    ros-${ROS_DISTRO}-navigation \
    ros-${ROS_DISTRO}-map-server \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-plugins \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-catkin \
    ros-${ROS_DISTRO}-roslaunch \
    ros-${ROS_DISTRO}-rospack \
    ros-${ROS_DISTRO}-costmap-2d \
    ros-${ROS_DISTRO}-global-planner \
    ros-${ROS_DISTRO}-dwa-local-planner \
    ros-${ROS_DISTRO}-base-local-planner \
    ros-${ROS_DISTRO}-navfn \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-tf2-tools

# 4. 安装 Python 依赖
echo -e "${BLUE}[INFO]${NC} 安装 Python 依赖..."
pip3 install --user --upgrade numpy matplotlib pyyaml opencv-python pillow shapely networkx jinja2

# 5. 初始化 rosdep 并安装工作区依赖
echo -e "${BLUE}[INFO]${NC} 初始化 rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || true
fi
rosdep update

echo -e "${BLUE}[INFO]${NC} 安装工作区 src 下的 rosdep 依赖..."
cd "$WORKSPACE"
rosdep install --from-paths src --ignore-src -r -y

echo -e "${GREEN}[SUCCESS]${NC} 所有依赖安装完成！"

# 询问是否编译
echo ""
read -p "是否现在编译项目？(y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${BLUE}[INFO]${NC} 开始编译..."
    cd "$WORKSPACE"
    catkin build tag_nav_description tag_nav_bringup tag_nav_localization tag_nav_planner
    echo -e "${GREEN}[SUCCESS]${NC} 编译完成！"
    echo -e "${BLUE}[INFO]${NC} 请执行: source $WORKSPACE/devel/setup.bash"
else
    echo -e "${YELLOW}[INFO]${NC} 跳过编译。您可以稍后手动编译："
    echo "  cd $WORKSPACE"
    echo "  catkin build tag_nav_description tag_nav_bringup tag_nav_localization tag_nav_planner"
fi