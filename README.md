# Marty Sim

Barebones, barely working simulator for AUV Marty.

Work in progress.

Contains scenario files that should be run with [stonefish_ros](https://github.com/patrykcieslak/stonefish_ros).    
This package contains a translation layer that should work out of the box with output from the TVMC and the rest of system. 

It should also output sensor data as expected by the system, along with a realistic amount of noise added on top.

## Setup (Refer to the actual stonefish repo for better instructions)

1. **Install Stonefish simulator:**
   ```bash
   git clone https://github.com/patrykcieslak/stonefish.git
   cd stonefish
   mkdir build && cd build
   cmake ..
   make -j$(nproc)
   sudo make install
   ```

2. **Clone stonefish_ros:**
   ```bash
   cd ~/ros_ws/src
   git clone https://github.com/patrykcieslak/stonefish_ros.git
   ```

3. **Build the workspace:**
   ```bash
   cd ~/ros_ws
   catkin_make
   source devel/setup.bash
   ```

4. **Run the simulator:**
  Use the launch files present for this stuff. Look into abi.launch or create your own launch file!



The simulator currently uses **AUV Marty** as the robot. The robot definition is imported from `data/scenarios/new_marty.scn`.

The scenario is loaded via `new_auv.scn` which includes the robot and arena setup.

## Test Publish Commands

Control the thrusters via the `/control/thrusters` topic using a `Float64MultiArray` with 7 values:
`[lb, lf, rb, rf, tl, tr, tb]` - Values in range [-1.0, 1.0]

Some example commands: 

```bash
rostopic pub /control/thrusters std_msgs/Float64MultiArray "data: [0.5, 0.5, 0.5, 0.5, 0.0, 0.0, 0.0]" --once

rostopic pub /control/thrusters std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5]" --once

rostopic pub /control/thrusters std_msgs/Float64MultiArray "data: [0.5, 0.5, 0.5, 0.5, 0.0, 0.0, 0.0]" -r 10
```
