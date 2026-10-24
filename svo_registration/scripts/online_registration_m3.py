#!/usr/bin/env python3
# FYP Phase 8 (M3): copy of online_registration_fyp.py that differs ONLY in the registration step,
# selected by registration_settings.registration_method:
#   point_to_plane (default): the M2 path, unchanged (twin crop + Open3D point-to-plane ICP).
#   distance_field          : C++ distance-field registration (module fyp_df, svo_registration/df_cpp;
#                             port of analysis/df_register.py) against the unsigned distance grid of the
#                             same twin (--df-grid <grid>.bin; header twin md5 verified against
#                             df_twin_md5 AND against the md5 of --reference-map).
# For distance_field, the ICP quantities used downstream are replaced by their distance-field analogues
# (docs/phase7_offline.md): fitness = fraction of points with untruncated D < max_correspondence_distance,
# inlier_rmse = RMS of D over those points, correspondences = (inlier point, normalised grad D at the
# corrected point) for inliers with |grad D| > 1e-9. Validity checks, the information heuristic and its
# normalisation/scale factors, topics, outputs and logging are the M2 code. The twin is still loaded
# (published on /city_twin for RViz) but not cropped, because no target cloud is needed.
# Information option df_information: heuristic (M2 formula, main) | hessian (J^T W J / sigma^2,
# [pos, rot], published as is; D4 ablation). Logging adds the columns method and iterations
# (iterations empty for point_to_plane: Open3D does not report them).
# Origin of online_registration_fyp.py (FYP copy of online_registration.py, original untouched; D3):
# Differences from the original:
#   1. Parameters (read from registration_settings, defaults = the original hard-coded values):
#        twin_voxel_size   (default 1.0 m;  0 or null = no downsampling of the twin)
#        twin_min_z        (default 0.0 m;  null = no z filter)
#        fitness_threshold (default 0.75)
#   2. Bug fixes: F1 arccos argument clipped to [-1, 1] (NaN passed the rotation check);
#                 F2 rows with |a_i x n_i| ~ 0 skipped in the information matrix (division by zero).
#   3. Optional per-keyframe CSV log (registration_settings.log_file; default off): point counts,
#      ICP fitness/rmse, correction size, accept/reject reason, and timings split into
#      preprocessing (local + twin crop/filter), ICP solve, and information-matrix computation.
#   4. Optional per-keyframe event dump (--event-dir; default off, Phase 6): one .npz per processed
#      keyframe with the local cloud as received and after the node's filters, the initial guess
#      (VIO keyframe pose in the map frame; ICP initialised at identity), the ICP transformation and
#      result pose, fitness/rmse/correspondences, the raw and final information matrices, verdict,
#      reason and timings. Written by a background thread after publishing (no effect on outputs).
# Everything else (topics, ICP settings, information heuristic, output message) is identical.
import argparse
import time
import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped
import transforms3d as t3d
import tf.transformations
from threading import Lock
import threading
import queue
import os
import hashlib
import yaml
from svo_msgs.msg import PoseWithCloud
from std_srvs.srv import Empty, EmptyResponse

DEFAULT_VOXEL_SIZE = 0.1
WORLD_FRAME_ID = "world"

class DFRegistrationResult:
    """The fields of Open3D's RegistrationResult that the node uses, for the distance-field branch."""

    def __init__(self, transformation, fitness, inlier_rmse, correspondence_set):
        self.transformation = transformation
        self.fitness = fitness
        self.inlier_rmse = inlier_rmse
        self.correspondence_set = correspondence_set


class CityMapRegistrator:

    def __init__(self, config_file, reference_map, df_grid=None):
        self.lock = Lock()
        print(f"READING FROM {reference_map}")
        self.df_grid_file = df_grid
        self.load_config(config_file,reference_map)
        self.init_ros_nodes()
        self.republish_service = rospy.Service('/visualize_city_twin', Empty, self.republish_city_twin)
        self.registration_estimate = np.eye(4)

    def republish_city_twin(self, request):
        rospy.loginfo("Re-publishing point clouds on demand.")
        self.publish_city_twin()
        return EmptyResponse()

    def publish_city_twin(self):
        point_cloud_msg = self.pc_to_ros_pointcloud2(self.city_twin)
        point_cloud_msg.header.stamp = rospy.Time.now()
        point_cloud_msg.header.frame_id = WORLD_FRAME_ID
        self.city_twin_publisher.publish(point_cloud_msg)

    def init_ros_nodes(self):
        rospy.init_node('map_registration_node', anonymous=True)

        self.w_svo_pose_point_est_sub = rospy.Subscriber("/svo/w_backend_pose_points", PoseWithCloud, self.svo_reg_callback, queue_size=1)
        self.registered_point_cloud_pub = rospy.Publisher("/svo/w_backend_points_registered", PointCloud2, queue_size=1)
        self.registration_T_w_b_pub = rospy.Publisher("/svo/w_registrated_pose", PoseWithCovarianceStamped, queue_size=1)
        self.city_twin_publisher = rospy.Publisher('/city_twin', PointCloud2, queue_size=1, latch=True)

    def load_config(self, config_file,reference_map):
        try :
            with open(config_file, 'r') as file:
                config = yaml.safe_load(file)['registration_settings']
        except FileNotFoundError:
            rospy.logerr(f"Configuration file not found: {config_file}")
            rospy.signal_shutdown("Shutting down due to missing config file.")
        except yaml.YAMLError as e:
            rospy.logerr(f"Error loading YAML configuration: {str(e)}")
            rospy.signal_shutdown("Shutting down due to YAML error.")

        self.valid = True
        self.scale_factor_position = config['scale_factor_position']
        self.scale_factor_rotation = config['scale_factor_rotation']
        self.max_correspondence_distance = config['max_correspondence_distance']
        self.inliers_rmse_thr = config['inliers_rmse_thr']
        self.max_translation_distance = config['max_translation_distance']
        self.power = config['information_matrix_power']
        self.max_rotation_distance = config['max_rotation_distance']
        self.size_bounding_box = config['size_bounding_box']
        # FYP parameters; defaults are the original hard-coded values.
        self.twin_voxel_size = config.get('twin_voxel_size', 1.0)
        self.twin_min_z = config.get('twin_min_z', 0.0)
        self.fitness_threshold = config.get('fitness_threshold', 0.75)
        self.log_file = config.get('log_file', None)
        self.event_dir = None
        self.log = None
        if self.log_file:
            self.log = open(self.log_file, 'w', buffering=1)
            self.log.write("stamp,n_raw,n_filtered,n_target,fitness,inlier_rmse,corr_t_m,corr_rot_deg,"
                           "valid,reject_reason,t_preproc_s,t_icp_s,t_info_s,method,iterations\n")

        # M3 settings (Phase 8); defaults select the unchanged M2 path.
        self.registration_method = config.get('registration_method', 'point_to_plane')
        if self.registration_method not in ('point_to_plane', 'distance_field'):
            raise ValueError(f"unknown registration_method {self.registration_method}")
        self.df_huber_delta = config.get('df_huber_delta', 0.05)
        self.df_max_iter = config.get('df_max_iter', 100)
        self.df_information = config.get('df_information', 'heuristic')
        if self.df_information not in ('heuristic', 'hessian'):
            raise ValueError(f"unknown df_information {self.df_information}")
        if self.registration_method == 'distance_field':
            import fyp_df
            self.fyp_df = fyp_df
            expected = config['df_twin_md5']
            with open(reference_map, 'rb') as f:
                twin_md5 = hashlib.md5(f.read()).hexdigest()
            if twin_md5 != expected:
                raise RuntimeError(f"reference map md5 {twin_md5} != df_twin_md5 {expected}")
            self.df_grid = fyp_df.DistanceGrid(self.df_grid_file, expected)  # verifies header, size, checksum, md5
            rospy.loginfo(f"distance field: {self.df_grid_file}, dims {self.df_grid.dims.tolist()}, voxel "
                          f"{self.df_grid.voxel}, {self.df_grid.memory_bytes() / 1e6:.1f} MB, delta {self.df_huber_delta}, "
                          f"max_iter {self.df_max_iter}, information {self.df_information}")

        self.city_twin = self.load_and_filter_point_cloud(reference_map)
        if self.twin_voxel_size:
            self.city_twin = self.city_twin.voxel_down_sample(voxel_size=self.twin_voxel_size)
        rospy.loginfo(f"twin: {len(self.city_twin.points)} points (voxel {self.twin_voxel_size}, min_z {self.twin_min_z}), "
                      f"normals {self.city_twin.has_normals()}")

    def load_and_filter_point_cloud(self, ply_file_path):
        try :
            point_cloud = o3d.io.read_point_cloud(ply_file_path)
        except FileNotFoundError:
            rospy.logerr(f"Reference map not found: {ply_file_path}")

        points = np.asarray(point_cloud.points)

        # Filter out points below z=twin_min_z (original: z=0, underground)
        if self.twin_min_z is None:
            mask = np.ones(len(points), dtype=bool)
        else:
            mask = points[:, 2] >= self.twin_min_z
        filtered_points = points[mask]

        # Create a new point cloud with filtered points
        filtered_point_cloud = o3d.geometry.PointCloud()
        filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)

        # If normals exist, filter and assign them to the new point cloud
        if point_cloud.has_normals():
            normals = np.asarray(point_cloud.normals)
            filtered_normals = normals[mask]
            filtered_point_cloud.normals = o3d.utility.Vector3dVector(filtered_normals)

        return filtered_point_cloud

    def calculate_information_matrix(self, source_points, target_normals):
        source_points = np.asarray(source_points)
        target_normals = np.asarray(target_normals)
        Apn = np.zeros((6,6))
        arr_normal = np.zeros((3,1))

        for a_i, n_i in zip(source_points, target_normals):
            arr_normal[0] += n_i[0]
            arr_normal[1] += n_i[1]
            arr_normal[2] += n_i[2]
            cross_prod = np.cross(a_i, n_i)
            norm = np.linalg.norm(cross_prod)
            if norm < 1e-12:  # F2: point parallel to its normal; skip instead of dividing by zero
                continue
            cross_prod = cross_prod / norm
            H_i = np.vstack([-cross_prod.reshape(-1, 1), -n_i.reshape(-1, 1)]).T
            Apn += H_i.T @ H_i

        # rotation and position were inversed
        Apn_correct_order = np.zeros((6,6))
        Apn_correct_order[:3,:3] = Apn[3:,3:]
        Apn_correct_order[3:,3:] = Apn[:3,:3]
        Apn_correct_order[:3,3:] = Apn[3:,:3]
        Apn_correct_order[3:,:3] = Apn[:3,3:]
        Apn_correct_order = np.linalg.matrix_power(Apn_correct_order, self.power)

        return Apn_correct_order

    def transformation_from_pose(self, pose_msg):

        q = pose_msg.pose.orientation
        quaternion = [q.w, q.x, q.y, q.z]
        rot_matrix = t3d.quaternions.quat2mat(quaternion)

        position = [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z]

        transformation = np.eye(4)
        transformation[:3, :3] = rot_matrix
        transformation[:3, 3] = position

        return transformation

    def create_pose_with_covariance_stamped(self, registration_est, informations, stamp, frame_id=WORLD_FRAME_ID):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = frame_id

        rotation_matrix = registration_est[:3, :3]
        translation_vector = registration_est[:3, 3]

        quaternion = tf.transformations.quaternion_from_matrix(
            np.vstack([np.hstack([rotation_matrix, [[0], [0], [0]]]), [0, 0, 0, 1]])
        )

        norm = np.linalg.norm(quaternion)
        if norm != 1:
            quaternion /= norm

        pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, pose_msg.pose.pose.position.z = translation_vector
        pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w = quaternion
        pose_msg.pose.covariance = tuple(informations.flatten())

        return pose_msg

    def pc_to_ros_pointcloud2(self, pointcloud):
        points = np.asarray(pointcloud.points, dtype=np.float32)
        header = Header(frame_id=WORLD_FRAME_ID)
        ros_cloud = point_cloud2.create_cloud_xyz32(header, points)
        return ros_cloud

    def filter_city_twin(self, T_world_body):
        # Bound the target point cloud to limit optimization time.
        min_point, max_point = T_world_body[:3, 3] - self.size_bounding_box, T_world_body[:3, 3] + self.size_bounding_box

        ref_points_np = np.asarray(self.city_twin.points)
        ref_normals_np = np.asarray(self.city_twin.normals)

        mask = np.all((ref_points_np >= min_point) & (ref_points_np <= max_point), axis=1)
        target_point_cloud = o3d.geometry.PointCloud()
        target_point_cloud.points = o3d.utility.Vector3dVector(ref_points_np[mask])
        target_point_cloud.normals = o3d.utility.Vector3dVector(ref_normals_np[mask])

        return target_point_cloud

    def filter_local_map(self, msg, T_world_body):
        input_points = np.asarray(
            list(point_cloud2.read_points(msg.point_cloud, skip_nans=True, field_names=("x", "y", "z")))
        )
        min_point, max_point = T_world_body[:3, 3] - self.size_bounding_box, T_world_body[:3, 3] + self.size_bounding_box
        filtered_points = input_points[np.all((input_points >= min_point) & (input_points <= max_point), axis=1)]

        filtered_pointcloud = o3d.geometry.PointCloud()
        filtered_pointcloud.points = o3d.utility.Vector3dVector(filtered_points)
        filtered_pointcloud, _ = filtered_pointcloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=0.5)

        return filtered_pointcloud.voxel_down_sample(voxel_size=0.1)

    def event_writer(self):
        # Phase 6: writes queued per-keyframe events; runs in its own thread.
        while True:
            ev = self.event_queue.get()
            if ev is None:
                return
            msg = ev.pop("msg")
            raw = np.asarray(list(point_cloud2.read_points(msg.point_cloud, skip_nans=True,
                                                           field_names=("x", "y", "z"))), dtype=np.float64)
            np.savez_compressed(os.path.join(self.event_dir, "kf_%d.npz" % msg.header.stamp.to_nsec()),
                                stamp_ns=np.int64(msg.header.stamp.to_nsec()), cloud_raw=raw, **ev)

    def svo_reg_callback(self, msg):
        with self.lock:
            t_start = time.perf_counter()
            T_world_body = self.transformation_from_pose(msg.pose)
            current_vio_pointcloud = self.filter_local_map(msg, T_world_body)
            if self.registration_method == 'point_to_plane':
                target_point_cloud = self.filter_city_twin(T_world_body)
                n_target = len(target_point_cloud.points)
            t_pre = time.perf_counter()

            # Registration step: the only part that differs between M2 and M3. Both branches provide
            # icp_result (transformation, fitness, inlier_rmse, correspondence_set) and target_point_cloud
            # (normals indexed by the correspondences) to the unchanged code below.
            df_res = None
            if self.registration_method == 'point_to_plane':
                icp_result = o3d.pipelines.registration.registration_icp(
                    source=current_vio_pointcloud,
                    target=target_point_cloud,
                    max_correspondence_distance=self.max_correspondence_distance,
                    init=np.identity(4),
                    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=25),
                )
            else:
                icp_result, target_point_cloud, df_res = self.register_distance_field(current_vio_pointcloud)
                n_target = 0  # no target cloud: the grid replaces it
            t_icp = time.perf_counter()

            self.valid, reason = self.validate_icp_result(icp_result)
            t_info = 0.0
            A_raw = np.full((6, 6), np.nan)
            A_final = np.full((6, 6), np.nan)

            if self.valid:
                reg_T_WB = icp_result.transformation @ T_world_body
                filtered_points = np.asarray(current_vio_pointcloud.points).copy()

                if df_res is not None and self.df_information == 'hessian':
                    # D4 ablation: J^T W J / sigma^2 in [pos, rot], published without the heuristic's
                    # normalisation and scale factors.
                    A_pn = self.fyp_df.information_hessian(df_res["H_wv"], df_res["sigma2"])
                    A_raw = A_pn.copy()
                    current_vio_pointcloud.transform(icp_result.transformation)
                else:
                    matched_target_indices = [tgt for _, tgt in icp_result.correspondence_set]
                    target_normals = np.asarray(target_point_cloud.normals)[matched_target_indices]
                    source_indices = [src for src, _ in icp_result.correspondence_set]
                    source_points = np.asarray(current_vio_pointcloud.points)[source_indices]

                    A_pn = self.calculate_information_matrix(source_points, target_normals)
                    A_raw = A_pn.copy()
                    current_vio_pointcloud.transform(icp_result.transformation)

                    A_pn = (A_pn/np.max(A_pn))* np.exp(-0.9 * icp_result.inlier_rmse)
                    A_pn[:3,:3] *= self.scale_factor_position*self.scale_factor_position
                    A_pn[3:6, 3:6] *= self.scale_factor_rotation*self.scale_factor_rotation
                    A_pn[:3, 3:6] *= self.scale_factor_rotation*self.scale_factor_position
                    A_pn[3:6,:3] *= self.scale_factor_rotation*self.scale_factor_position
                t_info = time.perf_counter() - t_icp
                A_final = A_pn.copy()
                pose_msg_w_b = self.create_pose_with_covariance_stamped(reg_T_WB, A_pn, msg.header.stamp)

                self.registration_T_w_b_pub.publish(pose_msg_w_b)
                registrated_pointcloud = self.pc_to_ros_pointcloud2(current_vio_pointcloud)
                registrated_pointcloud.header.stamp = msg.header.stamp
                registrated_pointcloud.header.frame_id = WORLD_FRAME_ID
                self.registered_point_cloud_pub.publish(registrated_pointcloud)

            if self.event_dir:
                filtered = filtered_points if self.valid else np.asarray(current_vio_pointcloud.points).copy()
                self.event_queue.put(dict(
                    msg=msg, cloud_filtered=filtered.astype(np.float64),
                    n_target=np.int64(n_target), registration_method=self.registration_method,
                    iterations=np.int64(df_res["iterations"] if df_res is not None else -1),
                    df_information=self.df_information if df_res is not None else "",
                    T_init_world_body=T_world_body, T_icp_init=np.identity(4),
                    T_icp=np.asarray(icp_result.transformation),
                    T_result_world_body=np.asarray(icp_result.transformation) @ T_world_body,
                    fitness=icp_result.fitness, inlier_rmse=icp_result.inlier_rmse,
                    n_correspondences=np.int64(len(icp_result.correspondence_set)),
                    information_raw=A_raw, information_final=A_final,
                    valid=bool(self.valid), reject_reason=reason,
                    t_preproc_s=t_pre - t_start, t_icp_s=t_icp - t_pre, t_info_s=t_info))

            if self.log:
                T = icp_result.transformation
                ang = np.degrees(np.arccos(np.clip((np.trace(T[:3, :3]) - 1) / 2, -1.0, 1.0)))
                self.log.write(f"{msg.header.stamp.to_sec():.9f},{msg.point_cloud.width * msg.point_cloud.height},"
                               f"{len(current_vio_pointcloud.points)},{n_target},"
                               f"{icp_result.fitness:.6f},{icp_result.inlier_rmse:.6f},{np.linalg.norm(T[:3, 3]):.6f},"
                               f"{ang:.4f},{int(self.valid)},{reason},{t_pre - t_start:.6f},{t_icp - t_pre:.6f},{t_info:.6f},"
                               f"{self.registration_method},{df_res['iterations'] if df_res is not None else ''}\n")

    def register_distance_field(self, cloud):
        """M3 registration step. Returns (icp_result-like object, cloud holding the gradient normals,
        raw fyp_df result). Same init (identity) and correction convention as the ICP branch."""
        pts = np.asarray(cloud.points)
        res = self.fyp_df.register(self.df_grid, pts, mu=None, delta=self.df_huber_delta, max_iter=self.df_max_iter)
        D, v, G = res["D_untrunc"], res["valid_untrunc"], res["G_untrunc"]
        inl = v & (D < self.max_correspondence_distance)
        gn = np.linalg.norm(G, axis=1)
        use = inl & (gn > 1e-9)
        normals = o3d.geometry.PointCloud()
        normals.points = o3d.utility.Vector3dVector(pts[use])
        normals.normals = o3d.utility.Vector3dVector(G[use] / gn[use, None])
        result = DFRegistrationResult(
            transformation=res["T_c"], fitness=float(inl.sum() / max(len(D), 1)),
            inlier_rmse=float(np.sqrt(np.mean(D[inl] ** 2))) if inl.any() else np.inf,
            correspondence_set=np.c_[np.flatnonzero(use), np.arange(int(use.sum()))])
        return result, normals, res

    def validate_icp_result(self, icp_result):
        reasons = []
        if icp_result.fitness < self.fitness_threshold:
            reasons.append("fitness")
        if icp_result.inlier_rmse > self.inliers_rmse_thr:
            reasons.append("rmse")
        if reasons:
            return False, "+".join(reasons)

        # The following 2 conditions are because we assume that the results given by the VIO
        # is not completely off, so we accept only results "close by"
        # If the result given by the VIO was too off, a realignment from world frame to
        # local frame would be necessary and we shouldn't return a value in any case.
        translation_norm = np.linalg.norm(icp_result.transformation[:3, 3])
        # F1: clip the arccos argument; rounding can push it above 1 (NaN would pass the check).
        cos_angle = np.clip((np.trace(icp_result.transformation[:3, :3]) - 1) / 2, -1.0, 1.0)
        rotation_angle = np.degrees(np.arccos(cos_angle))
        if translation_norm > self.max_translation_distance:
            reasons.append("translation")
        if rotation_angle > self.max_rotation_distance:
            reasons.append("rotation")
        if reasons:
            return False, "+".join(reasons)

        return True, ""


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Node to register local point cloud to the city twin.")
    parser.add_argument('--config-file', type=str)
    parser.add_argument('--reference-map', type=str)
    parser.add_argument('--log-file', type=str, default=None)
    parser.add_argument('--event-dir', type=str, default=None)
    parser.add_argument('--df-grid', type=str, default=None, help="distance grid .bin (registration_method distance_field)")
    # args = parser.parse_args()
    args, unknown = parser.parse_known_args()

    processor = CityMapRegistrator(args.config_file, args.reference_map, args.df_grid)
    processor.event_dir = None if args.event_dir in (None, "none") else args.event_dir
    if processor.event_dir:
        os.makedirs(processor.event_dir, exist_ok=True)
        processor.event_queue = queue.Queue()
        writer = threading.Thread(target=processor.event_writer, daemon=True)
        writer.start()
    if args.log_file and processor.log is None:
        processor.log = open(args.log_file, 'w', buffering=1)
        processor.log.write("stamp,n_raw,n_filtered,n_target,fitness,inlier_rmse,corr_t_m,corr_rot_deg,"
                            "valid,reject_reason,t_preproc_s,t_icp_s,t_info_s,method,iterations\n")
    rospy.loginfo("CityMapRegistrator node started. Waiting for messages...")
    try:
        pointcloud_thread = threading.Thread(target=processor.publish_city_twin)
        pointcloud_thread.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("PointCloudProcessor node shutdown.")
    if processor.event_dir:
        processor.event_queue.put(None)  # drain queued events before exiting
        writer.join(timeout=20)
