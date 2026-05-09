# Autonomous Drone-Rover Terrain Analysis — PX4 SITL + Gazebo Harmonic

Rowan University — Database Systems & Robotics Final Project

A simulated autonomous mission where a ground rover surveys terrain using GPS altitude data, identifies a flat landing zone, then signals a drone to take off, fly to the rover's location, and land nearby using lidar.

---

## System Requirements

- Ubuntu 24+
- ROS2 Jazzy
- Gazebo Harmonic (gz-sim 8)
- PX4 Autopilot (SITL build)
- Python 3.12

---

## Dependencies

### Python (install into venv)
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### ROS2 packages
```bash
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-actuator-msgs
```

---

## Setup

### 1. Clone and build PX4
```bash
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
make px4_sitl
```

### 3. Add to ~/.bashrc
```bash
export BUILD_DIR="$HOME/finalProjectDroneRobots/PX4-Autopilot/build/px4_sitl_default"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$BUILD_DIR/src/modules/simulation/gz_plugins:$BUILD_DIR/lib"
export GZ_PLUGIN_PATH="$BUILD_DIR/src/modules/simulation/gz_plugins:$BUILD_DIR/lib"
export LD_LIBRARY_PATH="$BUILD_DIR/src/modules/simulation/gz_plugins:$BUILD_DIR/lib:$LD_LIBRARY_PATH"
export GZ_SIM_RESOURCE_PATH="$HOME/.simulation-gazebo/models:$HOME/finalProjectDroneRobots/PX4-Autopilot/Tools/simulation/gz/models"
```

---

## Running

**Terminal 1 — Launch simulation**
```bash
export GZ_SIM_RESOURCE_PATH="$HOME/.simulation-gazebo/models:$HOME/finalProjectDroneRobots/PX4-Autopilot/Tools/simulation/gz/models"
cd ~/finalProjectDroneRobots/PX4-Autopilot
bash run.sh
```
Wait for: `Failsafe params set OK`

**Terminal 2 — ROS2 bridge**
```bash
source venv/bin/activate
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/rover_differential_2/command/motor_speed@actuator_msgs/msg/Actuators@gz.msgs.Actuators \
  /world/default/model/rover_differential_2/link/base_link/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat \
  /world/default/model/x500_lidar_front_1/link/base_link/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat \
  /world/default/model/x500_lidar_front_1/link/lidar_sensor_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /world/default/model/rover_differential_2/link/lidar_link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

**Terminal 3 — Run mission**
```bash
source venv/bin/activate
source /opt/ros/jazzy/setup.bash
python3 scriptLidar.py
```

---

## How It Works

1. Rover drives forward autonomously through a terrain field of mounds
2. GPS altitude is sampled to detect when the rover is on elevated terrain
3. Once 5 consecutive seconds of flat ground are detected, the rover stops and signals the drone
4. Drone arms, initializes offboard mode, and takes off to 20m
5. Drone flies to rover GPS coordinates
6. Drone lidar scans nearby offsets to find flat ground
7. Drone lands at confirmed flat zone next to rover

---