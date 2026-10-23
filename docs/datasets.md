# Datasets (Phase 1)

Verified on 2026-10-23. Data table approved by the user on 2026-10-23. Everything under `dataset/` is git-ignored. Disk: 190 GB free; `dataset/` = 34 GB.

## 1. EuRoC sequences

Each sequence folder holds the official ROS bag **and** the ASL zip (the zips have not been extracted, except V2_01_easy, which has an extracted `mav0/` folder).
Small files (sensor.yaml, ground-truth CSVs, pointcloud0) were extracted from the zips into `dataset/derived/asl_meta/<seq>/mav0/` (689 MB). The images were not extracted.
**No ASL→rosbag conversion is needed.**

| Sequence | Format | Bag duration | GT (state_groundtruth_estimate0) | Other GT | pointcloud0 | Notes |
|---|---|---|---|---|---|---|
| MH_01_easy | bag + zip | 186 s | yes, 36 382 rows, 181.9 s | leica0 (position) | no | `/leica/position` in bag |
| MH_02_easy | bag + zip | 151 s | yes, 150.0 s | leica0 | no | |
| MH_03_medium | bag + zip | 135 s | yes, 131.5 s | leica0 | no | |
| MH_04_difficult | bag + zip | 102 s | yes, 98.8 s | leica0 | no | cam0 2033 / cam1 2032 msgs |
| MH_05_difficult | bag + zip | 114 s | yes, 111.1 s | leica0 | no | |
| V1_01_easy | bag + zip | 144 s | yes, 28 712 rows, 143.6 s | vicon0 | yes (V1 scan) | `/vicon/firefly_sbx/firefly_sbx` in bag |
| V1_02_medium | bag + zip | 88 s | yes, 83.5 s | vicon0 | yes (V1 scan) | cam0 1710 / cam1 1711 msgs |
| V1_03_difficult | bag + zip | 109 s | yes, 104.7 s | vicon0 | yes (V1 scan) | |
| V2_01_easy | bag + zip + extracted ASL | 113 s | yes, 112.0 s | vicon0 (CSV only) | yes (V2 scan) | **no Vicon topic in bag**; GT from CSV only |
| V2_02_medium | bag + zip | 117 s | yes, 115.4 s | vicon0 (CSV only) | yes (V2 scan) | no Vicon topic in bag |
| V2_03_difficult | bag + zip | 116 s | yes, 114.8 s | vicon0 (CSV only) | yes (V2 scan) | no Vicon topic in bag; **cam0 drops 17.7 % of frames** (see §1.1) |

### 1.1 Camera frame drops (cam0, the camera used for mono VIO)
Computed from the `mav0/cam0/data.csv` timestamps against a nominal 20 Hz rate:

| Sequence | cam0 frames / expected | Missing | Longest gap |
|---|---|---|---|
| V2_03_difficult | 1922 / 2336 | **414 (17.7 %)** | 0.10 s (all drops are single frames) |
| all other 10 sequences | complete | 0 % | 0.05 s |

cam1 is complete in V2_03 (2336/2336). V2_03 stays in the evaluation; its cam0 drop rate must be reported next to its results.

### 1.2 Ground-truth source per sequence
- **V2_01/02/03: ground truth comes from `state_groundtruth_estimate0/data.csv` only.** The V2 bags have no Vicon topic.
- V1 and MH: same CSV for evaluation, for consistency, even though V1 bags carry `/vicon/...` and MH bags carry `/leica/position`.
- GT is used only for evaluation and the documented one-time initial alignment.

**Bag topics.** MH: `/cam0/image_raw`, `/cam1/image_raw`, `/imu0` (200 Hz), `/leica/position`.
V1/V2: `/cam0/image_raw`, `/cam1/image_raw` (20 Hz), `/imu0` (200 Hz), `/fcu/imu`, `/fcu/motor_speed` (`asctec_hl_comm/MotorSpeed`), and, for V1 only, `/vicon/firefly_sbx/firefly_sbx` (TransformStamped).
Use `/imu0`, not `/fcu/imu` (in V2, `/fcu/imu` is only ~10 Hz).

**Ground-truth format.** `state_groundtruth_estimate0/data.csv` has `timestamp[ns], p_RS_R (xyz), q_RS (wxyz), v_RS_R, b_w, b_a`. Its `T_BS` is identity, so it gives the **IMU/body frame B in the Vicon world frame V**.
The GT starts about 1 s after the bag starts.

## 2. Leica scans (pointcloud0): digital-twin candidates

| | V1 scan | V2 scan |
|---|---|---|
| File | `mav0/pointcloud0/data.ply`, ASCII PLY | same |
| Fields | x y z intensity diffuse_rgb (RGB all 0) | same |
| Points | 3 199 068 | 3 120 095 |
| Identical across the room's 3 sequences | yes (same md5) | yes (same md5) |
| Room extent (0.5–99.5 percentile) | x −4.37…3.68, y −3.47…4.95, z −0.02…4.10 m | x −5.04…3.33, y −3.63…4.36, z −0.05…4.08 m |
| Raw bbox (with stray points) | 73.3 × 31.2 × 14.6 m | 62.2 × 82.3 × 14.3 m |
| Stray points outside [−6,5]×[−5,6]×[−0.5,4.5] | 1 | 33 |
| NN spacing (median / p90) | 5.1 / 10.4 mm | 5.2 / 10.4 mm |
| Occupied 10 cm voxels | 38 533 (≈83 pts/voxel) | 38 781 (≈80 pts/voxel) |
| Normals | none (must be estimated for point-to-plane / signed distance) | none |

Sensor comment: "Pointcloud recorded with a Leica Nova MS50 from 6 different positions". The six station holes are visible in the top views.

### Scan frame findings
- `pointcloud0/sensor.yaml` gives `T_WR = I₄`, meaning the scan is expressed in the dataset **world frame**, the same frame as the Vicon ground truth. So nominally **M = V**, with the identity transform from the dataset's own metadata.
- Headless frame check (GT is used for visualisation only): all GT positions of all six sequences lie inside the scan bounds and inside the room walls in the top view. GT height is 0.6–2.9 m above a floor at z ≈ 0 m, with the ceiling at ≈4.1 m. The median distance from GT to the nearest scan point is 0.8–1.1 m. There are no signs of rotation, offset, or axis-convention errors.
- The minimum GT→scan distance is 0.07 m in all six sequences. This is plausibly the MAV resting on a scanned surface at take-off/landing (not verified).
- **The original `.ply` files are never modified.** Stray points are removed by a documented crop in Phase 5 preprocessing; the output goes to `dataset/derived/`.
- Phase 5 preprocessing needs: crop to the room box (removes 1/33 stray points), downsampling (5 mm spacing is far denser than needed), and normal estimation.

Figures: `docs/img/V{1,2}_scan_top.png`, `V{1,2}_scan_side_xz.png`, `V{1,2}_scan_side_yz.png`, `V{1,2}_framecheck_gt_overlay.png` (labelled FRAME CHECK ONLY). Numbers: `docs/img/phase1_scan_stats.txt`. Script: `analysis/phase1_scan_check.py`.

### Map independence
The scan is an independent Leica MS50 measurement. Its alignment to the Vicon frame comes from the dataset authors' calibration, not from the MAV trajectories. We do not use GT to build or align the map.

## 3. Oerlikon (upstream data)

**Map path override (required):** the launch file's default map path does not exist. Launch with
```
roslaunch svo_ros Oerlikon.launch reference_map:=/home/alpha/Desktop/twinforge/rpg_svo_pro_with_digital_twins/dataset/Oerlikon.ply
```

| Expected by | Path expected | Exists? |
|---|---|---|
| `svo_ros/launch/Oerlikon.launch` arg `reference_map` (default) | `$(find svo_registration)/maps/Oerlikon.ply` | **No**: `svo_registration/maps/` does not exist. The file is at `dataset/Oerlikon.ply` (152 MB, binary PLY, 3 111 643 pts, xyz + normals + rgb). Pass `reference_map:=<abs path>/dataset/Oerlikon.ply` at launch; no copy needed. |
| `online_registration.py --config-file` | `svo_registration/config/oerlikon.yaml` | yes |
| svo_node `calib_file`, rosparam, `gp_settings_file`, `map_reg_settings_file` | `svo_ros/param/Oerlikon/{calibration,mono_vio,gp_settings,gpose_settings}.yaml` | yes (all 4) |
| RViz config | `svo_ros/RVIZ/rviz_config_digital_twins.rviz` | yes |
| Loop-closure vocabulary `voc_GEN_8X4.yml.gz` | — | not needed: `runlc: False` |
| Rosbag (played manually) | — | `dataset/oerlikon.bag`, 399 s |

Bag topics vs launch: `/camera_2/image_raw` ✓ (`cam0_topic`), `/vio_module/imu_filtered` ✓ (`imu_topic`), `/gps` ✓ (`gp_topic`, PoseStamped, 399 msgs). Also `/groundtruth` (PoseStamped, 1996 msgs). The bag time starts at 135.97 s (simulator clock), while the launch sets `/use_sim_time` False. Phase 3 will check whether this matters.

Related: the `svo_registration/config/FM.yaml` `reference_map` points to an upstream author's home directory. It is unused by Oerlikon.launch.

## 4. Other findings relevant to later phases
- EuRoC VIO launch files already exist: `svo_ros/launch/Default/euroc_vio_{mono,stereo,gp_mono,gp_stereo}.launch` and `euroc_vio_mono_test.launch`; params in `svo_ros/param/Default/euroc_{mono,stereo}.yaml`. svo_benchmarking has EuRoC experiment configs. These will be inspected in Phase 2.
- Analysis tools (evo 1.31.1, Open3D 0.19.0) are in the venv `~/venvs/fyp`; see `docs/environment.md`.
- The pre-existing local branch `voxel` is **ignored by user decision**: it is not read, merged, or reused. All volumetric work starts fresh on `dev/volumetric-registration`.

## 5. Open questions
1. ~~Mono or stereo?~~ **Decided: mono-inertial (cam0 + `/imu0`)** for M1/M2/M3, matching Oerlikon. Stereo is optional later.
2. How did the EuRoC authors register the Leica scan to the Vicon frame, and with what accuracy? This bounds the achievable map-frame ATE. Check Burri et al. 2016.
3. ~~voxel branch~~: ignored (decided).
4. ~~V2 GT source~~: CSV only (§1.2).
