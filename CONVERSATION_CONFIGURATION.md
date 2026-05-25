# Conversation Setup

## Install

```bash
sudo apt update
sudo apt install -y python3-pip espeak-ng libportaudio2 portaudio19-dev
python3 -m pip install --user --break-system-packages soniox sounddevice
```

Check:

```bash
python3 - <<'PY'
import rclpy
import soniox
import sounddevice
print("ok")
PY
```

If `ros2 run` cannot import `soniox` or `sounddevice`:

```bash
export PYTHONPATH="$(python3 -m site --user-site):$PYTHONPATH"
```

## API Key

For the current terminal:

```bash
export SONIOX_API_KEY='your_real_soniox_key_here'
```

For permanent setup, add the same line to `~/.bashrc`:

```bash
nano ~/.bashrc
source ~/.bashrc
```

## Run

Build + source

```bash
export SONIOX_API_KEY='your_real_soniox_key_here' #ali v bashrc pa ga sourcas
```

Terminal 1:

```bash
ros2 run task2 speak
```

Terminal 2:

```bash
ros2 run task2 conversation
```

Terminal 3:

```bash
ros2 service call /converse_person msg_types/srv/ConversePerson "{gender: female}"
```

Male test:

```bash
ros2 service call /converse_person msg_types/srv/ConversePerson "{gender: male}"
```



## Microphone

test if microphone is visible: 

```bash
python3 -m sounddevice
```

