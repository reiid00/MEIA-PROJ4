# ROS2 PX4 Workspace Setup Guide

This guide covers the setup process for the ROS2 PX4 Workspace on Ubuntu 20.04.

## Prerequisites

Ensure that your system meets the following requirements:

- Ubuntu 20.04

## Required Software

Here is a list of all the software components that need to be installed:

- ROS 2 Foxy
- Gazebo
- PX4 Source
- Micro XRCE-DDS Agent & Client

## Installation Steps

### Outside of Current Repo

```bash
cd ..
```

### ROS 2 Foxy Installation

First, check your locale:

```bash
locale  # check for UTF-8
```

Now, set your locale to UTF-8 if it's not already set:

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
```

Install required software properties:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Update and install curl:

```bash
sudo apt update && sudo apt install curl -y
```

Get the ROS key:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the ROS repository:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Update and upgrade your system:

```bash
sudo apt update
sudo apt upgrade
```

Install ROS Foxy Desktop and other required tools:

```bash
sudo apt install ros-foxy-desktop python3-argcomplete
sudo apt install ros-dev-tools
```

For more details, visit the [official ROS 2 Foxy installation guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).

### Gazebo Installation

Install Gazebo ROS packages and other necessary dependencies:

```bash
sudo apt install ros-foxy-gazebo-ros-pkgs
sudo apt install ros-foxy-ros-core ros-foxy-geometry2
```

For more details, visit the [Gazebo ROS installation guide](http://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros).

### PX4 Source Installation

Clone the PX4 Autopilot repository and run the setup script:

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

For more details, visit the [PX4 source setup guide](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html#simulation-and-nuttx-pixhawk-targets).

### Setup Micro XRCE-DDS Agent & Client

Clone the Micro XRCE-DDS Agent repository and build it:

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

For more details, visit the [Micro XRCE-DDS setup guide](https://docs.px4.io/main/en/ros/ros2_comm.html#setup-micro-xrce-dds-agent-client).

## How to Run

1. Start MicrXRCEAgent in a new terminal tab:

    ```bash
    MicroXRCEAgent udp4 -p 8888
    ```

2. Start PX4 Environment in another terminal tab:

    ```bash
    cd PX4-Autopilot
    DONT_RUN=1 make px4_sitl gazebo
    Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m iris -n 4
    ```

3. Setup ROS2 PX4 Workspace in a third terminal tab:

    ```bash
    cd MEIA-PROJ4
    cd ROS2-PX4-Workspace
    colcon build
    source /opt/ros/foxy/setup.bash
    source install/local_setup.bash
    ```