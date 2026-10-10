# Drift-free Visual SLAM using Digital Twins

[![Tightly-coupled Fusion of Global Position Measurements in SVO Pro](doc/eyecatcher_with_youtube_logo.jpg)](https://youtu.be/gmHnhWYfuW0)

This repo contains a sliding-window optimization-based odometry system fusing visual, inertial and global pose measurements obtained by registering to a Digital Twin. 
It builds on top of the visual-inertial odometry algorithm [SVO Pro](https://github.com/uzh-rpg/rpg_svo_pro_open).

## Publication
If you use this code in an academic context, please cite the following [RAL 2025 paper](https://rpg.ifi.uzh.ch/docs/RAL24_Merat.pdf).

R. Merat (equal contribution), G. Cioffi (equal contribution), L. Bauersfeld, and D. Scaramuzza,
"**Drift-free Visual SLAM using Digital Twins**,"
Robotics and Automation Letters (RA-L). 2025.

```
@article{merat2025drift,
  title={Drift-free Visual SLAM using Digital Twins},
  author={Merat, Roxane and Cioffi, Giovanni and Bauersfeld, Leonard and Scaramuzza, Davide},
  journal={IEEE Robotics and Automation Letters},
  year={2025},
  publisher={IEEE}
}
```

## Install

The code builds and runs on

* Ubuntu 24.04 with ROS 2 Jazzy (tested, also via the provided Docker image)

The ROS 1 (Melodic/Noetic) version of this repository is available in the git history
(commit `d641322`).

### Install dependencies

Install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
(`ros-jazzy-desktop`), then the build tools and system dependencies:
```sh
sudo apt install python3-colcon-common-extensions python3-rosdep python3-pip \
  libceres-dev libeigen3-dev libgeographiclib-dev libgflags-dev libgoogle-glog-dev \
  libopencv-dev libopengv-dev libsuitesparse-dev libyaml-cpp-dev \
  python3-transforms3d python3-yaml python3-scipy python3-matplotlib \
  ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-pcl-conversions \
  ros-jazzy-tf2-ros ros-jazzy-rosbag2-storage-mcap
```
The online registration uses [Open3D](https://www.open3d.org/), which is not packaged for
Ubuntu 24.04. `rosbags` is used to convert ROS 1 bags (such as the Oerlikon dataset) to ROS 2:
```sh
# Keep numpy < 2: the ROS 2 Python bindings shipped with Ubuntu 24.04 are built against numpy 1.x.
pip3 install --break-system-packages --ignore-installed psutil
pip3 install --break-system-packages "numpy<2" open3d rosbags
```

Ceres Solver >= 2.1 is required (the backend uses the `ceres::Manifold` API; Ubuntu 24.04 ships 2.2).
The remaining third-party libraries (`fast`, `minkindr`, `eigen_checks`, `DBoW2`) are built by the
wrapper packages in [third_party/](third_party/), which download the upstream sources at a pinned
commit during the first build (pass `-DFETCHCONTENT_SOURCE_DIR_<NAME>=<path>` to build offline).

### Clone and compile

```sh
mkdir -p ~/svo_ws/src && cd ~/svo_ws/src
git clone https://github.com/uzh-rpg/rpg_svo_pro_with_digital_twins.git
cd ~/svo_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
Add `-DBUILD_TESTING=OFF` to skip building the unit tests.
`svo_global_map` (GTSAM based, disabled by default as in the original) is skipped by colcon
because of its `COLCON_IGNORE` file.

### Docker

[docker/Dockerfile](docker/Dockerfile) provides the complete Ubuntu 24.04 + ROS 2 Jazzy
environment:
```sh
docker build -t svo-digital-twin:jazzy -f docker/Dockerfile docker
docker run -it --rm --net=host -v ~/svo_ws:/ws -w /ws svo-digital-twin:jazzy bash
# inside the container
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash
```

### Troubleshooting

Refer to [SVO Pro](https://github.com/uzh-rpg/rpg_svo_pro_open#troubleshooting).

## Instructions
To launch the program on our custom dataset download the rosbag and the city twin files from [here](https://download.ifi.uzh.ch/rpg/svo_with_digital_twins)
and place them in `dataset/` (`dataset/oerlikon.bag` and `dataset/Oerlikon.ply`).

The dataset is a ROS 1 bag. Convert it once to a ROS 2 bag:
```sh
rosbags-convert --src dataset/oerlikon.bag --dst dataset/oerlikon_ros2 --dst-storage mcap
```
Then run everything, including the bag playback, with
```sh
ros2 launch svo_ros Oerlikon.launch.py \
  reference_map:=$PWD/dataset/Oerlikon.ply bag:=$PWD/dataset/oerlikon_ros2
```
or leave out `bag:=...` and play the bag in another terminal (`ros2 bag play dataset/oerlikon_ros2`).
Other launch arguments: `rviz:=false` (headless), `registration:=false`, `rate:=0.5`,
`calib_file:=`, `params_file:=` (see `ros2 launch svo_ros Oerlikon.launch.py --show-args`).
When using your own data, adjust the topic names in the launch file.

#### That's it already you are set!

This will launch :
- svo_ros with the global pose accepted as input to the optimization problem
- svo_registration, which will register the local point cloud to the global city twin.
- rviz2

The estimated pose in the world (city twin) frame is published on `/svo/w_backend_pose_imu`,
the VIO estimate on `/svo/vio_backend_pose_imu`, and the poses from the registration on
`/svo/w_registrated_pose`. To record them for evaluation:
```sh
ros2 bag record -o svo_result /svo/w_backend_pose_imu /svo/vio_backend_pose_imu /svo/w_registrated_pose /groundtruth /gps
```
For trajectory evaluation, [rpg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation)
can be used; the scripts in `svo_registration/Evaluation` convert result bags to its format
(they read ROS 2 bags through `vikit_py.rosbag_utils`).

### Notes on the ROS 2 port

- Parameters: the files loaded as node parameters (e.g. `param/Oerlikon/mono_vio.yaml`) use the
  ROS 2 `ros__parameters` format; nested ROS 1 names such as `T_world_imuinit/qx` are now
  `T_world_imuinit.qx`. Calibration and settings files read by SVO itself are unchanged.
- The ROS 1 nodelet is replaced by the component `svo_ros::SvoComponent`.
- ROS 2 topic names cannot start a token with a digit, so per-camera topics are now
  `/svo/image_with_features_0`, `/svo/pose_cam_0`, `/svo/dense_input_0`, and the GPS simulator
  publishes `.../satellite/pos/sat_<n>` and `.../gnss/pseudorange/sat_<n>`.
- `/svo/w_backend_pose_points` (the input of the registration) is now published whenever
  someone subscribes to it; in ROS 1 it was only published if the backend point clouds were
  also subscribed (e.g. by RViz).
- The GPS simulator (`satellite_simulator`): `ros2 launch satellite_simulator gnss_simulator.launch.py`.
- The rqt plugin: `ros2 run rqt_svo rqt_svo`.
- Offline tools (`svo_registration/Dataset`, `svo_registration/Evaluation`, `svo_benchmarking`, `scripts/python`)
  read and write ROS 2 bags through `vikit_py.rosbag_utils`.

### Known issues (inherited from the original code)

- After a tracking failure, SVO re-initializes with a new backend that does not re-use the
  world (GPS / city twin) alignment, so `/svo/w_*` outputs and the registration stop for the rest
  of the run. On the Oerlikon dataset tracking normally does not fail, but it can happen
  occasionally, since the pipeline is multi-threaded and not deterministic.
- `test_estimator` (svo_ceres_backend) fails at marginalization, as in the ROS 1 version, and
  `vikit_cameras_tests` needs `test/data/test_mask.png`, which is not part of the repository.

## Credits

The VIO system used in this repo is based on [SVO Pro](https://github.com/uzh-rpg/rpg_svo_pro_open). Check [SVO Pro](https://github.com/uzh-rpg/rpg_svo_pro_open) for the full list of aknowledgments.

## License

The code is licensed under GPLv3. For commercial use, please contact the authors at cioffi@ifi.uzh.ch and sdavide@ifi.uzh.ch.

Check [SVO Pro](https://github.com/uzh-rpg/rpg_svo_pro_open) for licenses of the external dependencies.
