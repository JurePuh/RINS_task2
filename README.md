# RINS Task 2 — Autonomous Robot Pipeline

**Team** · Blaž Bergant · Jure Puh · Peter Žaucer

ROS 2 project for RINS 2026 Task 2: a TurtleBot that explores two rooms, talks to people, runs perception tasks, and reports results to the CTO.

**Full write-up:** [Report/report.pdf](Report/report.pdf)

---

## Overview

The robot starts in the first room and searches for people. When it finds someone, it greets them and asks for a task. Four possible jobs:

| Task | What the robot does |
|------|---------------------|
| **Count rings** | Record every ring seen (position + color) |
| **Inspect barrels** | Record barrels; approach horizontal ones and check for leaks |
| **Red / green cell anomaly** | Drive along the matching colored belt and inspect tiles for defects |

A yellow line in the first room is a **no-go zone** — the planner never crosses it.

After the first room is done, the robot enters the second room, follows a blue line through intersections, and stops when it reaches the **CTO**. That triggers the final report: ring counts, barrel table (with leak images), and tile inspection results (with anomaly masks), each tagged with who requested the task.

---

## Architecture

Perception, geometry, conversation, and actuation are separate ROS 2 nodes. A central **movement** node (behaviour tree via `py_trees_ros`) orchestrates them over topics and services.

| Area | Nodes / modules |
|------|-----------------|
| **Perception** | Face detection (YOLO), ring detection (HSV + Hough), barrel & leak detection (PCL RANSAC), blue-line vision |
| **Recognition & speech** | Face classification (FaceNet embeddings), conversation (Soniox STT), TTS (`speak`) |
| **Anomaly** | SuperSimpleNet tile inspection on the belt |
| **Navigation** | Nav2 + keep-out map, wall-normal approach goals, PD belt / line following |
| **Control** | Behaviour tree in `movement` |

Packages live under `src/`:

- `task2` — Python nodes (movement, faces, rings, conversation, anomaly, …)
- `barrel_leak_cpp` — C++ barrel & leak detector
- `msg_types` — custom messages and services
- `dis_tutorial3` / `dis_tutorial7` — simulation, maps, TurtleBot launch

---

## Setup

Assumes a working ROS 2 environment (e.g. Jazzy/Humble), `colcon`, Gazebo, Nav2, and the RInS tutorial dependencies.

### System packages

```bash
sudo apt update
sudo apt install -y \
    espeak-ng \
    libportaudio2 \
    portaudio19-dev \
    ros-jazzy-py-trees \
    ros-jazzy-py-trees-ros \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    libpcl-dev
```

### Python packages

```bash
pip install \
    "numpy<2" \
    opencv-python \
    torch torchvision \
    ultralytics \
    facenet-pytorch \
    fpdf2 \
    sounddevice \
    soundfile \
    soniox
```

### Build

```bash
source /opt/ros/jazzy/setup.bash   # or your distro
colcon build --symlink-install
source install/setup.bash
```

### Environment

Add to `~/.bashrc` (then `source` it):

```bash
export QSG_RENDER_LOOP=basic
export SONIOX_API_KEY='your_soniox_key'
```

---

## Running

Each command in its own terminal (source ROS + `install/setup.bash` first):

```bash
# Terminal 1 — Zenoh RMW
ros2 run rmw_zenoh_cpp rmw_zenohd

# Terminal 2 — simulation + Nav2
ros2 launch dis_tutorial7 sim_turtlebot_nav.launch.py

# Terminal 3 — all nodes except movement
ros2 launch task2 not_movement.launch.py

# Terminal 4 — behaviour tree
ros2 run task2 movement
```

Debug a node with:

```bash
ros2 run task2 movement --ros-args --log-level movement:=debug
```

---

## Report & team

Methods, ROS graphs, results, and work split are in the PDF:

→ **[Report/report.pdf](Report/report.pdf)**

| Member | Focus |
|--------|--------|
| Blaž Bergant | Movement / behaviour tree, blue line following |
| Jure Puh | Face detection & classification, anomaly detection |
| Peter Žaucer | Ring & barrel detection, speech |
