# AI Chair Docking

AI Chair Docking is a ROS 2 Jazzy-based autonomous wheelchair docking and navigation system. The project uses occupancy maps, Nav2 planning and control servers, behavior trees, and custom ROS 2 control scripts to enable autonomous navigation and docking functionality.


## Repository Structure

```text
AI-Chair-Docking/
├── occupancy_map/
│   ├── map.yaml
│   ├── nav2_params.yaml
│   ├── ros2_controller.py
│   ├── simple_robot.urdf
│   └── ...
├── README.md
└── ...
```

---

# Requirements

## Software

- Ubuntu 24.04
- ROS 2 Jazzy
- Navigation2 (Nav2)
- RViz2
- Python 3

## Required ROS Packages

```bash
sudo apt install \
ros-jazzy-navigation2 \
ros-jazzy-nav2-bringup \
ros-jazzy-tf2-ros \
ros-jazzy-rviz2 \
ros-jazzy-robot-state-publisher
```

---

# Setup Instructions

The navigation stack is launched across multiple terminals.

---

## Terminal 1 — Start Map Server

```bash
source /opt/ros/jazzy/setup.bash

ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/home/lab-user/Documents/AiChair-Docking_Group/AI-Chair-Docking/occupancy_map/map.yaml \
  -p use_sim_time:=False
```

---

## Terminal 2 — Configure and Activate Map Server

```bash
source /opt/ros/jazzy/setup.bash

ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

---

## Terminal 3 — Publish Map → Odom Transform

```bash
source /opt/ros/jazzy/setup.bash

ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --yaw 0 --pitch 0 --roll 0 \
  --frame-id map \
  --child-frame-id wizard/odom
```

---

## Terminal 4 — Publish Odom → Base Link Transform

```bash
source /opt/ros/jazzy/setup.bash

ros2 run tf2_ros static_transform_publisher \
  --x 1.7 --y 0 --z 0 \
  --yaw 0 --pitch 0 --roll 0 \
  --frame-id wizard/odom \
  --child-frame-id wizard/base_link
```

---

## Terminal 5 — Launch RViz2

```bash
source /opt/ros/jazzy/setup.bash

rviz2
```

---

## Terminal 6 — Start Planner Server

```bash
source /opt/ros/jazzy/setup.bash

ros2 run nav2_planner planner_server --ros-args \
  --params-file /home/lab-user/Documents/AiChair-Docking_Group/AI-Chair-Docking/occupancy_map/nav2_params.yaml \
  -p use_sim_time:=False
```

---

## Terminal 7 — Configure and Activate Planner Server

```bash
source /opt/ros/jazzy/setup.bash

ros2 lifecycle set /planner_server configure
ros2 lifecycle set /planner_server activate
```

---

## Terminal 8/9 — OPTIONAL:  IF MAP NOT SHOWING IN RVIZ2!!

If the robot or map is not visible in RViz2:

### Map → Odom

```bash
source /opt/ros/jazzy/setup.bash

ros2 run tf2_ros static_transform_publisher \
  --x -0.4 --y 1.9 --z 0 \
  --yaw 0 --pitch 0 --roll 0 \
  --frame-id map \
  --child-frame-id wizard/odom
```

### Odom → Base Link

```bash
source /opt/ros/jazzy/setup.bash

ros2 run tf2_ros static_transform_publisher \
  --x 1.7 --y 0 --z 0 \
  --yaw 0 --pitch 0 --roll 0 \
  --frame-id wizard/odom \
  --child-frame-id wizard/base_link
```

---

## Terminal 10 — Start Controller Server

```bash
source /opt/ros/jazzy/setup.bash

ros2 run nav2_controller controller_server --ros-args \
  --params-file /home/lab-user/Documents/AiChair-Docking_Group/AI-Chair-Docking/occupancy_map/nav2_params.yaml \
  -p use_sim_time:=False
```

---

## Terminal 11 — Configure and Activate Controller Server

```bash
source /opt/ros/jazzy/setup.bash

ros2 lifecycle set /controller_server configure
ros2 lifecycle set /controller_server activate
```

---

## Terminal 12 — Start BT Navigator

```bash
source /opt/ros/jazzy/setup.bash

ros2 run nav2_bt_navigator bt_navigator --ros-args \
  --params-file ~/Documents/AiChair-Docking_Group/AI-Chair-Docking/occupancy_map/nav2_params.yaml \
  -p use_sim_time:=False \
  -p robot_base_frame:=wizard/base_link \
  -p global_frame:=map \
  -p odom_frame:=wizard/odom
```

---

## Terminal 13 — Start Behavior Server

```bash
source /opt/ros/jazzy/setup.bash

ros2 run nav2_behaviors behavior_server --ros-args \
  --params-file /home/lab-user/Documents/AiChair-Docking_Group/AI-Chair-Docking/occupancy_map/nav2_params.yaml \
  -p use_sim_time:=False
```

---

## Terminal 14 — Configure and Activate Behavior Server

```bash
source /opt/ros/jazzy/setup.bash

ros2 lifecycle set /behavior_server configure
ros2 lifecycle set /behavior_server activate
```

---

## Terminal 15 — Configure and Activate BT Navigator

```bash
source /opt/ros/jazzy/setup.bash

ros2 lifecycle set /bt_navigator configure
ros2 lifecycle set /bt_navigator activate
```

If navigation stops working:

```bash
ros2 lifecycle set /controller_server configure
ros2 lifecycle set /controller_server activate
```

---

## Terminal 16 — Start Custom Controller

```bash
python3 occupancy_map/ros2_controller.py
```

---

## Terminal 17 — Publish Robot State

```bash
source /opt/ros/jazzy/setup.bash

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(cat /home/lab-user/Documents/AiChair-Docking_Group/AI-Chair-Docking/occupancy_map/simple_robot.urdf)"
```

---

## Terminal 18 — Monitor Robot Motion

```bash
source /opt/ros/jazzy/setup.bash

ros2 topic echo /cmd_vel
```

---

---

# RViz2 Configuration

After launching RViz2, configure the following displays to properly visualize the map, robot, and navigation path.

## 1. Set Fixed Frame

Under **Global Options**:

- Fixed Frame: `map`

---

## 2. Add Map Display

1. Click **Add** → **Map**
2. Set the following properties:

| Property | Value |
|-----------|---------|
| Topic | `/global_costmap/costmap` |
| Reliability Policy | `Best Effort` |
| Durability Policy | `Transient Local` |

---

## 3. Add Path Display

1. Click **Add** → **Path**
2. Set:

| Property | Value |
|-----------|---------|
| Topic | `/plan` |

---

## 4. Add Pose Display

1. Click **Add** → **Pose**
2. Set:

| Property | Value |
|-----------|---------|
| Topic | `/goal_pose` |

This allows you to visualize navigation goals.

---

## 5. Add TF Display

1. Click **Add** → **TF**

This will display the transform tree between:

- `map`
- `wizard/odom`
- `wizard/base_link`

---

## 6. Add Robot Model Display

1. Click **Add** → **RobotModel**
2. Set:

| Property | Value |
|-----------|---------|
| Description Source | `Topic` |
| Description Topic | `/robot_description` |

The wheelchair model should now appear in RViz.

---

## Expected RViz Displays

Your RViz display panel should contain:

- Map
- Path
- Pose
- TF
- RobotModel

with **Global Options → Fixed Frame = `map`**.

# Manual Path Planning

To manually generate a path:

```bash
source /opt/ros/jazzy/setup.bash

ros2 action send_goal \
  /compute_path_to_pose \
  nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 1.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

# Troubleshooting

## Map Not Appearing

- Verify `/map_server` is active.
- Ensure RViz Fixed Frame is set to `map`.
- Confirm the path to `map.yaml` is correct.

## Robot Not Appearing

- Verify TF transforms are running.
- Launch `robot_state_publisher`.
- Confirm RViz displays are configured correctly.

## Navigation Fails

Verify all Nav2 nodes are active:

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
```

Expected state:

```text
active [3]
```

---
