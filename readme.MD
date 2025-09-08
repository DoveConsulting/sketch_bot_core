# Sketch Bot

## Requirements
- ROS Humble

## Installation
1. Clone all required ROS packages into your colcon workspace:
```bash
cd /path/to/colcon_ws/src
git clone -b sketch_bot https://github.com/DoveConsulting/ar4_ros_driver.git
git clone https://github.com/DoveConsulting/sketch_bot_ar_manipulator_ikfast_plugin.git
git clone https://github.com/DoveConsulting/sketch_bot_support.git
git clone https://github.com/DoveConsulting/sketch_bot_core.git
```
2. Install ROS dependencies:
```bash
sudo apt update

cd /path/to/colcon_ws
sudo rosdep init
rosdep update
rosdep install --from-path src -yr --ignore-src
```

3. Build and source workspace:
```bash
cd /path/to/colcon_ws
colcon build --symlink-install
source install/setup.bash
```

## Run Sketch demo
1. Launch Robot with MoveIt:
```bash
ros2 launch sketch_bot_core demo.launch.py
```
2. In a separate terminal, run:
```bash
ros2 launch sketch_bot_core sketch.launch.py
```