FROM nvidia/cuda:12.2.2-cudnn8-devel-ubuntu22.04
# CUDA11.2, cudnn8.1.1

# Maintainer Information
LABEL maintainer="wengkunduo@gmail.com"

RUN apt-get update

# 關閉互動模式
ENV DEBIAN_FRONTEND=noninteractive

## NVIDIA GraphicsCard parameter
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all

# 接收外部參數
ARG USER=initial
ARG UID=1000
ARG GROUP=initial
ARG GID=1000

# Switch apt source to Taiwan's mirror
# RUN sed -i 's@archive.ubuntu.com@tw.archive.ubuntu.com@g' /etc/apt/sources.list
RUN sed -i 's@archive.ubuntu.com@ftp.tku.edu.tw/@g' /etc/apt/sources.list

# Update and Install Packages
RUN apt-get update && apt-get install -y \
    software-properties-common \
    at-spi2-core \
    htop sudo git wget curl \
    byobu zsh \
    terminator \
    dbus-x11 libglvnd0 libgl1 libglx0 libegl1 libxext6 libx11-6 libxtst6 libinih-dev\
    nano vim \
    gnome-terminal libcanberra-gtk-module libcanberra-gtk3-module \
    python3-wheel python3-pip python3-dev python3-setuptools \
    libopencv-dev \
    openssh-server \
    cmake && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*


#=========================================================
#    Install ROS2 Humble
#=========================================================
# 添加ROS2源
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list
# 安裝ROS2 Humble
RUN apt-get update && apt-get install -y ros-humble-desktop
RUN apt-get update && apt-get install -y python3-colcon-common-extensions
RUN apt update && apt install ros-humble-vision-msgs

# DDS
RUN apt update && apt install -y ros-humble-rmw-cyclonedds-cpp

# 創建一個名為initial的用戶和用戶組
RUN groupadd -g $GID $GROUP && \
    useradd -m -u $UID -g $GID -s /bin/bash $USER && \
    echo "${USER} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${USER} && \
    chmod 0440 /etc/sudoers.d/${USER}

RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USER}/.bashrc

RUN apt-get update && apt-get install -y ros-humble-diagnostic-updater
RUN apt update && apt install -y ros-humble-dynamixel-sdk*

# RealSense SDK2.0
RUN sudo mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    tee /etc/apt/sources.list.d/librealsense.list
RUN apt-get update
RUN apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

# rviz map
RUN apt-get update && apt-get install -y ros-humble-navigation2 \
    ros-humble-nav2-bringup

#=========================================================
#    Installing python packages 
#=========================================================
# ROBOTIS DYNAMIXEL
# RUN pip3 install pynput



#=========================================================
#    Setting up Python PIP mirror for Taiwan
#=========================================================
# USER ${USER}
# RUN mkdir -p /home/${USER}/.pip && \
#     echo "[global]\nindex-url = https://pypi.nctu.edu.tw/simple" > /home/${USER}/.pip/pip.conf
# USER root

#=========================================================
#    YOLOv7
#=========================================================
RUN apt-get install unzip
RUN pip3 install numpy tqdm pandas seaborn unzip wandb
RUN pip3 install torch==2.0.1 torchvision==0.15.2 torchaudio==2.0.2
RUN pip3 install tensorflow==2.15.0


#=========================================================
#    Setting up Python PIP mirror for Taiwan
#=========================================================
# USER ${USER}
# RUN echo "[global]\nindex-url = https://pypi.tuna.tsinghua.edu.cn/simple" > /home/${USER}/.pip/pip.conf
# USER root

# # DDS
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# Timezone
RUN echo "tzdata tzdata/Areas select Asia" | debconf-set-selections && \
    echo "tzdata tzdata/Zones/Asia select Taipei" | debconf-set-selections && \
    apt-get update && apt-get install -y tzdata && \
    ln -snf /usr/share/zoneinfo/$TIME_ZONE /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/${USER}/.config/terminator/

WORKDIR /home/${USER}
USER ${USER}

RUN mkdir work
WORKDIR /home/${USER}/work

CMD ["terminator"]
USER $USER


