# residual-learning

ROS2 package for generating predefined PWM-based trajectories on leo rover. This repo is meant to be cloned and built directly — no need to manually create the package, edit `setup.py`, or wire up entry points by hand. That setup work is already done.

## Prerequisites

- ROS2 installed and sourced (`source /opt/ros/<distro>/setup.bash`)
- `colcon` build tools installed

## Setup

Clone this repo directly into the `src` folder of your workspace (create the workspace first if you don't have one):

```bash
mkdir -p ~/leo_ws/src
cd ~/leo_ws/src
git clone https://github.com/doctordoom54/residual-learning.git traj_gen
```

Build from source:

```bash
cd ~/leo_ws
colcon build --symlink-install --packages-select traj_gen
source install/setup.bash
```

Add the source step to your `.bashrc` so it runs automatically in every new terminal:

```bash
echo "source ~/leo_ws/install/setup.bash" >> ~/.bashrc
```

### A note on file permissions

ROS2 needs node scripts to be executable to run them with `ros2 run`. Git preserves the executable bit when files are committed correctly, so this usually carries over automatically when you clone. Quick way to check:

```bash
ls -l src/traj_gen/traj_gen/pwm_cmd.py
```

If the permissions string starts with `-rwxr-xr-x` (note the `x`s), you're good to go. If it starts with `-rw-r--r--` (no `x`), make it executable manually:

```bash
chmod +x src/traj_gen/traj_gen/*.py
```
Navigate to `~/leo_ws` then rebuild with `colcon build --symlink-install --packages-select traj_gen`.

## Package structure

```
traj_gen/
├── traj_gen/
│   ├── __init__.py
│   ├── pwm_cmd.py            # main node: sends predefined PWM commands, generates trajectory
│   ├── pwm8node.py           # pwm node to generate figure 8 trajectory
│   ├── leo_debug.py          # debug utilities
│   └── spiral_pwm_comm...    # predefined PWM command set for spiral trajectory shown in fig below
├── resource/
├── test/
├── package.xml
├── setup.py
└── setup.cfg
```

## Usage

In a new terminal, run the main trajectory node:

```bash
ros2 run traj_gen pwm_cmd
```

This sends the predefined PWM duty cycle commands and should (ideally) produce a trajectory like the one below:

<img width="1822" height="1730" alt="image" src="https://github.com/user-attachments/assets/ea1679ec-650a-45fc-b677-a6c5387d16c3" />



## Debugging

List active topics:

```bash
ros2 topic list
```

Inspect what's being published on a given topic:

```bash
ros2 topic echo /topic_name
```

Check the rate at which a topic is publishing:

```bash
ros2 topic hz /topic_name
```
