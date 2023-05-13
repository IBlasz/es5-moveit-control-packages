# es5-moveit-control-packages

## 1. Download and install
Download ros packages for MoveIt on Ubuntu operating system.
```bash
$ sudo apt-get install ros-noetic-desktop-full
$ sudo apt install ros-noetic-moveit
```

Compile
```bash
$ catkin_make
```

## 2. Set up an environment
```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
```

Clone packages from github and build your catkin workspace

```bash
$ git clone https://github.com/IBlasz/es5-moveit-control-packages.git
$ cd ~/catkin_ws
$ catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
$ catkin build
```

Source all setup.bash files to set up your enviroment.
```bash
# System configure ROS environment variables automatically every time you open a ternimal
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
