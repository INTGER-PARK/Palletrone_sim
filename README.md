This first version is developed by <https://github.com/Ryung-coding>

## Logging And MATLAB Workflow

### 1. Build

```bash
source /opt/ros/humble/setup.bash
cd /home/parkjeongsu/ros2_project/Palletrone_sim
colcon build --executor sequential
source install/setup.bash
```

### 2. Record All ROS 2 Topics

Run the simulation, then record every topic:

```bash
source /opt/ros/humble/setup.bash
source /home/parkjeongsu/ros2_project/Palletrone_sim/install/setup.bash

ros2 bag record -a -o bags/bag_all_01 \
  --compression-mode file \
  --compression-format zstd
```

If you use `ros2 launch palletrone_cmd arm_launch.py`, bag recording starts automatically and writes to:

```text
/home/parkjeongsu/ros2_project/Palletrone_sim/bags/bag_all_YYYYmmdd_HHMMSS
```

This records all topics including:
- `/palletrone_state`
- `/cmd`
- `/att_cmd`
- `/wrench`
- `/input`
- `/actuator_debug/servo_qpos`
- `/actuator_debug/requested_servo`
- `/actuator_debug/applied_bldc`
- `/actuator_debug/requested_bldc`

### 3. Convert Bag To CSV

Use the helper script:

```bash
source /opt/ros/humble/setup.bash
source /home/parkjeongsu/ros2_project/Palletrone_sim/install/setup.bash

python3 scripts/bag_to_csv.py bags/bag_all_01 --out-dir csv_out
```

This generates per-topic CSV files such as:
- `csv_out/palletrone_state.csv`
- `csv_out/cmd.csv`
- `csv_out/att_cmd.csv`
- `csv_out/wrench.csv`
- `csv_out/input.csv`
- `csv_out/actuator_debug__servo_qpos.csv`

### 4. Plot In MATLAB

From the workspace root:

```matlab
run("matlab/plot_tracking.m")
```

The MATLAB script creates these figures when the corresponding CSV files exist:
- Position tracking: `/cmd` desired vs `/palletrone_state` real
- Attitude tracking: `/att_cmd` desired vs `/palletrone_state` real
- Servo tracking: requested vs measured servo angle
- BLDC requested vs applied thrust
- Wrench command history
- Allocator input command history

### 5. Notes

- `AttitudeCmd` is logged in degrees.
- `/palletrone_state.rpy` is logged in radians, and converted to degrees in MATLAB.
- `wrench` and `input` currently do not have a direct measured feedback topic pair, so they are plotted as command histories rather than desired-vs-real overlays.

## Code Structure

```text
Sim_palletrone/
└── src/
    ├── palletrone_cmd/
    │   ├── CMakeLists.txt
    │   ├── launch/
    │   │   ├── arm_launch.py
    │   │   └── pt_launch.py
    │   ├── package.xml
    │   └── src/
    │       ├── attitude_sweep_cmd.cpp
    │       └── position_cmd.cpp
    │
    ├── palletrone_controller/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── src/
    │       ├── allocator_controller.cpp
    │       └── wrench_controller.cpp
    │
    ├── palletrone_interfaces/
    │   ├── CMakeLists.txt
    │   ├── msg/
    │   │   ├── AttitudeCmd.msg
    │   │   ├── Cmd.msg
    │   │   ├── Input.msg
    │   │   ├── PalletroneState.msg
    │   │   └── Wrench.msg
    │   └── package.xml
    │
    └── plant/
        ├── plant/
        │   ├── __init__.py
        │   ├── plant.py
        │   └── __pycache__/
        │       ├── __init__.cpython-310.pyc
        │       └── plant.cpython-310.pyc
        ├── resource/
        ├── test/
        ├── setup.cfg
        └── setup.py
