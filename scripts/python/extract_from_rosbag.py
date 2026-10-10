"""
Extract vio estimates from a ROS 2 bag
"""

import argparse
import os

import numpy as np
from vikit_py.rosbag_utils import read_messages, stamp_to_sec


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag_fn", type=str, required=True)
    parser.add_argument("--traj_out_fn", type=str, default='stamped_traj_estimate.txt')
    parser.add_argument("--pose_topic", type=str, default='/svo/vio_backend_pose_imu')
    parser.add_argument("--pose_topic_type", type=str, default='PoseWithCovarianceStamped')
    args = parser.parse_args()

    # load data
    traj = []  # [ts x y z qx qy qz qw]

    print('Reading topic %s from %s' % (args.pose_topic, args.bag_fn))

    if True:  # ROS 2 bag
        for (topic, msg, ts) in read_messages(args.bag_fn, topics=[args.pose_topic]):
            if topic == args.pose_topic:
                if args.pose_topic_type == 'PoseWithCovarianceStamped':
                    t = stamp_to_sec(msg.header.stamp)
                    px = msg.pose.pose.position.x
                    py = msg.pose.pose.position.y
                    pz = msg.pose.pose.position.z
                    qx = msg.pose.pose.orientation.x
                    qy = msg.pose.pose.orientation.y
                    qz = msg.pose.pose.orientation.z
                    qw = msg.pose.pose.orientation.w
                    traj_i = np.array([
                        t,
                        px, py, pz,
                        qx, qy, qz, qw])
                    traj.append(traj_i)

                elif args.pose_topic_type == 'PoseStamped':
                    t = stamp_to_sec(msg.header.stamp)
                    px = msg.pose.position.x
                    py = msg.pose.position.y
                    pz = msg.pose.position.z
                    qx = msg.pose.orientation.x
                    qy = msg.pose.orientation.y
                    qz = msg.pose.orientation.z
                    qw = msg.pose.orientation.w
                    traj_i = np.array([
                        t,
                        px, py, pz,
                        qx, qy, qz, qw])
                    traj.append(traj_i)

    traj = np.asarray(traj)

    # save
    traj_out_fn = os.path.join(os.path.dirname(args.bag_fn), args.traj_out_fn)
    np.savetxt(traj_out_fn, traj, header='ts x y z qx qy qz qw', fmt='%.6f')#, comments='')
    print('-- saved to %s' % traj_out_fn)

