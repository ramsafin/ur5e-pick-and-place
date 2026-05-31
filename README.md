# UR5e Pick and Place

ROS 2 **Jazzy** simulation of a Universal Robots **UR5e** with a **Robotiq 2F-85** gripper performing pick-and-place in **Gazebo (GZ Sim)**. The primary demo uses **MoveIt2** for motion planning.

**Requirements:** Ubuntu 24.04, ROS 2 Jazzy, display or WSLg for Gazebo GUI (headless mode available).

The colcon workspace root is this repository (packages live under `src/`).

---

## Repository layout

```
ur5e-pick-and-place/
├── src/
│   ├── ur5e_description/      # Unified URDF, ros2_control, initial positions
│   ├── ur5e_moveit_config/    # MoveIt2 SRDF, kinematics, controller mapping
│   ├── ur5e_bringup/          # Gazebo / MoveIt launch files, world, pick params
│   ├── ur5e_pick_and_place/   # Pick nodes, validation, smoke scripts
│   └── robotiq_description/   # Vendored Robotiq 2F-85 meshes + xacro
├── deps.repos                 # Upstream reference for robotiq_description
├── requirements.txt           # Pip-only deps (ikpy, opencv, pytest)
├── rosdep/                    # Custom rosdep rules for pip packages
└── README.md
```

### Dependency specification

| Kind | Where declared | Installed by |
|------|----------------|--------------|
| ROS / apt packages | `src/*/package.xml` | `rosdep install` |
| Workspace packages | `package.xml` `<exec_depend>` | `colcon build` |
| Git sources (optional) | `deps.repos` | `vcs import` |
| Pip-only tools | [`requirements.txt`](requirements.txt) | `pip install -r requirements.txt` |

Do **not** maintain a parallel manual `apt install` list for packages already in `package.xml` — that drifts from rosdep.

| Package | Role |
|---------|------|
| `ur5e_description` | Single robot xacro (mock or Gazebo), `ros2_controllers.yaml` |
| `ur5e_moveit_config` | SRDF, kinematics, MoveIt controller mapping |
| `ur5e_bringup` | Launches, `camera_world.sdf`, `pick_targets.yaml` |
| `ur5e_pick_and_place` | MoveIt pick nodes, `validate_config`, `go_home` |
| `robotiq_description` | Robotiq URDF macros and meshes |

---

## Install and build

### 1. ROS 2 Jazzy and build tools

```bash
sudo apt update
sudo apt install -y git python3-pip python3-colcon-common-extensions python3-rosdep
sudo rosdep init   # skip if already initialized
rosdep update
sudo apt install -y ros-jazzy-desktop
```

Add to `~/.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Projects/ur5e-pick-and-place/install/setup.bash
```

### 2. Project dependencies (from package.xml)

ROS dependencies are declared in each package’s [`package.xml`](src/ur5e_bringup/package.xml) and installed with **rosdep** (not a hand-maintained apt list).

```bash
cd ~/Projects/ur5e-pick-and-place
rosdep update
bash scripts/rosdep_install.sh
# Or: export PYTHONWARNINGS=ignore::DeprecationWarning  # silences apt rosdep pkg_resources warning
#     rosdep install --from-paths src --ignore-src -r -y
```

Optional: clone Robotiq from upstream instead of the vendored copy:

```bash
# vcs import src < deps.repos
```

Pip-only extras (experimental ikpy demo, tests):

```bash
pip3 install -r requirements.txt --break-system-packages
```

### 3. Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Projects/ur5e-pick-and-place
colcon build --symlink-install
source install/setup.bash
```

Verify:

```bash
ros2 pkg list | grep ur5e
# ur5e_bringup, ur5e_description, ur5e_moveit_config, ur5e_pick_and_place, robotiq_description
```

---

## Quick start (MoveIt pick-and-place)

**Terminal 1 — simulation + MoveIt:**

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ur5e_bringup gz_moveit.launch.py
```

Wait until:

- Gazebo shows the table, red box, and UR5e arm
- Log shows `Startup homing complete` (arm moves to HOME)
- Controllers are active: `ros2 control list_controllers`
- `move_group` is running (starts after homing completes)

**Terminal 2 — run pick task** (after stack is ready):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ur5e_bringup pick_moveit.launch.py
```

**Headless:**

```bash
ros2 launch ur5e_bringup gz_moveit.launch.py headless:=true
ros2 launch ur5e_bringup pick_moveit.launch.py
```

**Vision-guided pick (single launch):**

```bash
ros2 launch ur5e_bringup vision_pick.launch.py
```

**MoveIt RViz demo (mock hardware, no Gazebo):**

```bash
ros2 launch ur5e_bringup moveit_mock.launch.py
```

---

## Launch reference

| Launch file | What it starts |
|-------------|----------------|
| `ur5e_bringup gz_moveit.launch.py` | Gazebo + controllers + bridge + MoveIt + startup homing |
| `ur5e_bringup gz_sim.launch.py` | Gazebo + controllers + bridge (no MoveIt) |
| `ur5e_bringup pick_moveit.launch.py` | MoveIt pick node only (sim must be running) |
| `ur5e_bringup vision_pick.launch.py` | Full stack + vision-guided pick |
| `ur5e_bringup moveit_mock.launch.py` | RViz MoveIt demo + mock controllers |

---

## Configuration

| File | Purpose |
|------|---------|
| `src/ur5e_bringup/config/pick_targets.yaml` | Pick/place poses, gripper, vision scale |
| `src/ur5e_description/config/initial_positions.yaml` | HOME joint pose (sim + mock) |
| `src/ur5e_description/config/ros2_controllers.yaml` | Shared ros2_control controller config |

Generate flat URDF for ikpy (optional):

```bash
bash $(ros2 pkg prefix ur5e_description)/share/ur5e_description/scripts/generate_urdf.sh
```

---

## Validation

```bash
ros2 run ur5e_pick_and_place validate_config
bash src/ur5e_pick_and_place/scripts/validate_launches.sh
bash src/ur5e_pick_and_place/scripts/smoke_test_sim.sh mock
```

---

## Migration from `ur5e_ws` layout

| Old | New |
|-----|-----|
| `ur5e_ws/` workspace root | Repository root |
| `ur5e_gazebo_demo` | `ur5e_bringup` + `ur5e_pick_and_place` |
| `pick_moveit.launch.py` | `ur5e_bringup gz_moveit.launch.py` |
| `ur5e_sim.launch.py` | `ur5e_bringup gz_sim.launch.py` |
| `config/controllers.yaml` | `ur5e_description/config/ros2_controllers.yaml` |
| `config/home_positions.yaml` | `ur5e_description/config/initial_positions.yaml` |

---

## Troubleshooting

| Problem | What to try |
|---------|-------------|
| Pick hangs on gripper | Wait until all controllers are **active**; check `/clock` is publishing |
| `move_group` / pick rejected | Wait for `Startup homing complete` before `pick_moveit.launch.py` |
| `ur_description` not found | Re-run `rosdep install --from-paths src --ignore-src -r -y` |
| ikpy / vision import errors | `pip3 install -r requirements.txt` |
| `rosdep` pkg_resources DeprecationWarning | Harmless; use `bash scripts/rosdep_install.sh` or `PYTHONWARNINGS=ignore::DeprecationWarning` |
| Gazebo window missing (WSL2) | Use `headless:=true` |
| Gripper fingers desync | Rebuild after mimic fix; run `validate_config` |

---

## Known limitations

- Vision pixel scale may need per-setup calibration.
- Table collision is not in the MoveIt scene.
- ikpy demo uses generated `ur5e_full.urdf`; prefer MoveIt for pick-and-place.
