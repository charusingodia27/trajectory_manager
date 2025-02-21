# Trajectory Manager - ROS Package

## Overview

`trajectory_manager` is a ROS package for visualizing and saving the trajectory of an Autonomous Mobile Robot (AMR).It consists of two ROS nodes:

1. **Trajectory Saver**

   - Subscribes to `/odom` to collect the robot's trajectory.
   - Publishes trajectory markers in RViz.
   - Saves trajectory data to a file (`CSV` format) via a ROS service.

2. **Trajectory Reader**

   - Reads saved trajectory data from a file.
   - Publishes trajectory markers for visualization in RViz.

---

## Prerequisites
Before running this package, ensure you have the following installed:

- ROS Noetic
- TurtleBot3 Simulation packages:
  ```bash
  sudo apt update
  sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations ros-noetic-turtlebot3-teleop
  ```
- RViz for visualization:
  ```bash
  sudo apt install ros-noetic-rviz
  ```
- Ensure your workspace is properly sourced:
  ```bash
  source ~/catkin_ws/devel/setup.bash
  ```

---

## Installation

### 1. Clone the Repository

```bash
cd ~/catkin_ws/src
git clone <your-repo-url> trajectory_manager
```

### 2. Build the Package

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## Usage

### 1. Run the TurtleBot3 Simulation

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_fake turtlebot3_fake.launch
```

### 2. Move the Robot Using Keyboard

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

**Control Keys:**

```
W - Move Forward
S - Move Backward
A - Turn Left
D - Turn Right
X - Stop
```

### 3. Start the Trajectory Saver

```bash
rosrun trajectory_manager trajectory_saver
```

This node will record and visualize the trajectory in RViz.

### 4. Save the Trajectory

```bash
rosservice call /save_trajectory "{filename: '/home/$USER/trajectory.csv', duration: 10.0}"
```

This saves the last 10 seconds of the robot's trajectory.

### 5. Read and Visualize Saved Trajectory

```bash
rosrun trajectory_manager trajectory_reader_publisher
```

This node reads `trajectory.csv` and publishes markers in RViz.

---

## Visualizing in RViz

1. Open RViz:

```bash
rviz
```

2. Set Fixed Frame: `odom`
3. Add a New Display:
   - Click `Add` → `By Topic`
   - Select `/trajectory_markers` as a MarkerArray
4. Markers will appear as green spheres.

---

## Debugging & Troubleshooting

### 1. Check If `/odom` is Publishing Data

```bash
rostopic echo /odom
```

If values remain zero, move the robot using teleoperation.

### 2. Verify Saved Trajectory File

```bash
cat /home/$USER/trajectory.csv
```

You should see:

```
0.1, 0.0, 0.0
0.2, 0.1, 0.0
0.3, 0.2, 0.0
```

### 3. Check If Markers Are Being Published

```bash
rostopic echo /trajectory_markers
```

If empty, ensure the reader node is running.

### 4. Fix Duplicate Markers in RViz

If RViz shows "Adding marker multiple times," restart RViz and ensure marker IDs are unique in `trajectory_reader_publisher.cpp`.

---

## Project Structure

```
trajectory_manager/
│── launch/
│   ├── trajectory_saver.launch
│   ├── trajectory_reader.launch
│
│── src/
│   ├── trajectory_saver.cpp
│   ├── trajectory_reader_publisher.cpp
│
│── srv/
│   ├── SaveTrajectory.srv
│
│── CMakeLists.txt
│── package.xml
│── README.md
```

---

## Contributing

Feel free to submit issues or pull requests to improve this package.
