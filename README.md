# Project 2 — Workspace Setup

This README only covers setting up the workspace. It assumes you already have a typical ROS 2 development environment configured (ROS 2, `colcon`, Gazebo, Nav2, the RInS `dis_tutorial*` dependencies, etc.).

## 1. System packages (apt)

```bash
sudo apt update
sudo apt install -y \
    espeak-ng \
    libportaudio2 \
    portaudio19-dev \
    ros-jazzy-py-trees \
    ros-jazzy-py-trees-ros
```

- `libportaudio2` + `portaudio19-dev` + `espeak-ng` are needed for the conversation/speak nodes (microphone + TTS).
- `ros-jazzy-py-trees-ros` + `ros-jazzy-py-trees` are needed for the movement behaviour tree node.

## 3. Pip packages

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

Notes:
- `ultralytics` is used for YOLO detection (the repo ships a `yolov8n.pt` weights file at the workspace root).
- `facenet-pytorch` is used by the face classifier.
- `soniox` + `sounddevice` are used by the conversation node for speech.
- `py_trees` / `py_trees_ros` drive the movement behaviour tree and are installed via apt (step 1), not pip.

Test the mic is visible:

```bash
python3 -m sounddevice
```

## 4. Building

From the workspace root:

```bash
source /opt/ros/humble/setup.bash    # or your ROS distro
source ~/venvs/rins/bin/activate
colcon build --symlink-install
source install/setup.bash
```

## 5. `~/.bashrc` additions

Add these to `~/.bashrc` so every new terminal is ready to go:

```bash
# Make qt use single-threaded so gazebo doesnt crash
export QSG_RENDER_LOOP=basic

# Soniox API key for the conversation node
export SONIOX_API_KEY='your_real_soniox_key_here'

# Only if `ros2 run` can't import soniox / sounddevice from the user site:
# export PYTHONPATH="$(python3 -m site --user-site):$PYTHONPATH"
```

Then `source ~/.bashrc`.

## 6. Running

Each command goes in its own terminal (source ROS + the workspace's `install/setup.bash` in every one first):

```bash
# Terminal 1 — Zenoh RMW daemon
ros2 run rmw_zenoh_cpp rmw_zenohd

# Terminal 2 — simulation + Nav2
ros2 launch dis_tutorial7 sim_turtlebot_nav.launch.py

# Terminal 3 — everything except the movement node
clear && ros2 launch task2 not_movement.launch.py

# Terminal 4 — movement / behaviour tree
ros2 run task2 movement
```

### Debug logging

Append `--ros-args --log-level <node>:=debug` to any node to turn on debug logs, e.g.:

```bash
ros2 run task2 movement --ros-args --log-level movement:=debug
```

