
Drone autonomous planning, perception and navigation using ROS, Pixhawk (PX4) and Jetson Nano

# Jetson Nano Setup

## 1. Flash the sd card
- Download the image `jetson-nano-2gb-jp461-sd-card-image.zip`
- Flash it to the sd card using balena etcher
- Plug the sd card into the jetson nano
- Connect the USB-C port to power supply and the Micro-USB port to the laptop.

## 2. Login and Setup
- Find the COM port using device manager $\rightarrow$ Ports (usually `USB Serial Device - COM12`)
- Open Putty and connect to the COM port
- Go through the OEM setup and then reboot. Username = `jetson`, Password = `nano`
- Now ssh into the nano with `ssh jetson@192.168.55.1`
- Connect phone via USB and turn on USB tethering (or) Connect an ethernet cable
- Run the following commands
```bash
sudo apt update
sudo apt upgrade -y
```

## 3. Install ROS
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-ros-base
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
```

## 4. Pixhawk Setup
```bash
sudo apt install ros-melodic-mav* -y
cd Downloads
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh

# Give USB permissions
sudo usermod -a -G tty jetson
sudo usermod -a -G dialout jetson

```
- Find the vendor and product id. 
- To do this first run `lsusb` and now unplug the pixhawk and run `lsusb` again. 
- See what changed. For example it is `Bus 003 Device 005: ID 26ac:0011`
- Now vendor id is '26ac' and product id is '0011'
- Create a udev rule 
```bash
sudo nano /etc/udev/rules.d/99-pixhawk.rules
# Add the following line into the file (remember to change the ids)
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", SYMLINK+="ttyPixhawk"
# Save and close the file. Now Reboot.
```
- Create a launch file with the changed default parameter

```bash
roscd mavros
cd launch
sudo cp px4.launch pixhawk.launch
# Now edit the pixhawk.launch file and change the 'fcu_url' parameter
sudo nano pixhawk.launch
# Set 'fcu_url' to '/dev/ttyPixhawk'
# To run the ROS node, run the above created launch file
roslaunch mavros pixhawk.launch
```

## 5. Jetson Inference Installation
```bash
sudo apt-get update
sudo apt-get install git cmake libpython3-dev python3-numpy
git clone --recursive https://github.com/dusty-nv/jetson-inference
cd jetson-inference
mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig
```

# PX4 setup

## Environment
```bash
mkdir px4
cd px4
# Clone the repository
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
# Install dependencies
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
# Try the above command again if it fails
```

Put the following in `.bashrc`
```bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/pravardhan/px4/PX4-Autopilot/build/px4_sitl_default/build_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/pravardhan/px4/PX4-Autopilot/Tools/sitl_gazebo/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/pravardhan/px4/PX4-Autopilot/build/px4_sitl_default/build_gazebo
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/pravardhan/px4/PX4-Autopilot:/home/pravardhan/px4/PX4-Autopilot/Tools/sitl_gazebo
```
Start the simulation using
```bash
roslaunch px4 mavros_posix_sitl.launch
```

## Documentation

[Frames](https://mavlink.io/en/messages/common.html#MAV_FRAME)

[Mavros](http://wiki.ros.org/mavros)

[Tutorials](https://github.com/Intelligent-Quads/iq_tutorials)

[Intelligent Quads](https://www.youtube.com/c/IntelligentQuads/videos)
