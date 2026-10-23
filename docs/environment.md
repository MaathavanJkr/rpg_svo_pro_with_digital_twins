# Environment

## System
- Ubuntu 20.04, ROS Noetic (`/opt/ros/noetic`), system Python 3.8.10 (`/usr/bin/python3`), shell zsh.
- The user's `~/.local` site-packages already holds Open3D 0.19.0, numpy 1.24.4, scipy 1.10.1 and pip 25.0.1. This was not modified.

## Analysis venv: `~/venvs/fyp` (created 2026-10-23)

Used for all evaluation and analysis scripts (evo, Open3D, plotting). It is not used to build or run the catkin workspace.

| Package | Version | Note |
|---|---|---|
| evo | 1.31.1 | newest release supporting Python 3.8 |
| open3d | 0.19.0 | installed in the venv (same version as `~/.local`) |
| numpy | 1.24.4 | last release for Python 3.8 |
| scipy | 1.10.1 | |
| matplotlib | 3.7.5 | |
| pandas | 2.0.3 | evo dependency |
| rosbags | 0.9.23 | evo dependency (pure-Python bag reader) |

### How it was created
`python3-venv` is not installed (Debian strips `ensurepip`; installing it needs sudo), so the venv was bootstrapped without it:
```bash
mkdir -p ~/venvs
python3 -m venv --without-pip --system-site-packages ~/venvs/fyp
# install pip into the venv with the existing user-level pip
python3 -m pip --python ~/venvs/fyp/bin/python install --ignore-installed "pip==25.0.1"
```
**`--system-site-packages`** keeps the apt-installed ROS Python dependencies visible (`rospkg`, `catkin_pkg`, `yaml`, `Cryptodome`, `gnupg` in `/usr/lib/python3/dist-packages`).

**`~/.local` is hidden inside the venv.** These lines were appended to `~/venvs/fyp/bin/activate`, with a matching `unset` in `deactivate()`:
```bash
export PYTHONNOUSERSITE=1
```
Without this, the venv would also import the user's `~/.local` packages, and versions would depend on what happens to be installed there.

Then:
```bash
source ~/venvs/fyp/bin/activate
pip install "evo==1.31.1" "open3d==0.19.0"
```
Size: 1.7 GB. On first run, evo created `~/.evo/settings.json`.

### Usage
```bash
source /opt/ros/noetic/setup.zsh      # ROS first (adds /opt/ros/noetic/lib/python3/dist-packages to PYTHONPATH)
source ~/venvs/fyp/bin/activate
```

### Verified (2026-10-23)
| Check | venv | system python |
|---|---|---|
| `import rospy, rosbag, rospkg, catkin_pkg, yaml` | ✓ (from `/opt/ros/noetic/...`) | ✓ |
| Read `V1_01_easy.bag` via `rosbag.Bag` (107 819 msgs) | ✓ | ✓ |
| `rosbag info` CLI | ✓ | ✓ |
| `evo_ape --help` | ✓ | not installed (intended) |
| numpy / open3d resolved from | venv site-packages | `~/.local` |

### Caveat for running ROS nodes
Python ROS nodes started by `roslaunch` use their shebang. If a node uses `#!/usr/bin/env python3`, it picks up whichever `python3` is first on `PATH`, which is the venv python if the venv is active in that terminal. **Run `roslaunch` in a terminal without the venv active**, so online nodes keep using the same system/`~/.local` packages they were developed with. Use the venv only for offline evaluation and analysis.

## Building the catkin workspace (important)
`~/.zshrc` runs `brew shellenv`, which puts Linuxbrew (`/home/linuxbrew/.linuxbrew/bin`, including its own `ld` 2.47) first on `PATH`. Linking `svo_node` with that `ld` fails with hundreds of undefined references from system libraries (gdal, armadillo, opencv, yaml-cpp). A failed link also **deletes the previous binary**. **Always build with `scripts/build.sh`** (from 2026-10-23). It removes Linuxbrew from `PATH` for the build only, checks that `ld` is `/usr/bin/ld` (GNU ld 2.34), runs `catkin build` with the given arguments, and prints the `svo_node` / `libsvo_ros.so` timestamps and md5:
```bash
scripts/build.sh svo_ros --no-deps     # one package
scripts/build.sh                        # whole workspace
```
Your shell configuration is not changed. The 2026-10-23 13:15 rebuild (frame-event logger) was done with the same commands by hand, before the script existed.
