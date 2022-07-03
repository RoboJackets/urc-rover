# urc-software [![CI Status Badge](https://github.com/RoboJackets/urc-software/actions/workflows/ci.yml/badge.svg)](https://github.com/RoboJackets/urc-software/actions)


Welcome to the RoboJackets URC software repo! This document will give you a brief description of the repo's layout and an overview of the repo.

[![Software Lead](https://img.shields.io/badge/Software%20Lead-Aidan%20Stickan-green?color=EEB211&logo=data:image/png;base64,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)](https://github.com/a-stickan)

## Folder Structure

 * **urc_analysis**
    *Nodes used for scientific analysis*
 * **urc_gazebo**
    *Helper nodes used for simulation purposes*
* **urc_manipulation**
    *Collection of nodes used for the robotic arm*
* **urc_msgs**
    *Custom ROS messages used in the various ROS packages*
 * **urc_navigation**
    *Collection of nodes that form our navigation stack*
 * **urc_perception**
    *Collection of nodes that form our perception stack*
 * **urc_platform**
    *Nodes that are platform specific and used to communicate with the hardware, ie. IMU, joystick and motor controller. Also contains all unit tests.*
 * **urc_util**
    *A collection of utility nodes and classes*
 * **documents**
    *Research and design documents.*

## Installation Guide

### 1. Make sure you are running Ubuntu 22.04!

### 2. Install ROS2

```bash
sudo apt install software-properties-common
```
```bash
sudo add-apt-repository universe
```
```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
```bash
sudo apt update
```
```bash
sudo apt upgrade
```
```bash
sudo apt install ros-humble-desktop-full
```

### 3. Install Colcon
   
```bash
sudo apt install python3-colcon-common-extensions
```

### 4. bashrc Setup

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
```bash
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```
```bash
source ~/.bashrc
```

### 5. Create a colcon environment

```bash
cd <path to where you want to keep the project>
```
```bash
mkdir -p colcon-urc/src
```

### 6. Clone the repository into the colcon environment
```bash
cd colcon-urc/src
```
```bash
git clone https://github.com/RoboJackets/urc-software.git --recursive
```

### 7. Build your workspace
```bash
cd ..
```
```bash
colcon build --symlink-install
```

### 8. Source your environment
```bash
. install/setup.bash
```

### 9. Install and run rosdep
 Make sure to call the 'rosdep install' command from the colcon workspace directory (/colcon-urc)!
```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```


## Contributing
Join our slack [here!](https://robojackets.slack.com/)
