import argparse
import os

import numpy as np
from vikit_py.rosbag_utils import read_messages, BagWriter, sec_to_stamp
from geometry_msgs.msg import PoseStamped


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--groundtruth_fn", type=str, required=True)
    parser.add_argument("--bag_fn", type=str, required=True)
    parser.add_argument("--out_dir", type=str, required=True)
    parser.add_argument("--gt_freq_hz", type=float, default=200., 
        help='Freq. [hz] of ground truth')
    parser.add_argument("--gp_freq_hz", type=float, default=20., 
        help='Freq. [hz] of simulated global position measurments')
    parser.add_argument("--gp_std", type=float, default=0.2, 
        help='Standard deviation of the Gaussian distribution used to simulate gp noise [m]')
    args = parser.parse_args()

    gt = np.loadtxt(args.groundtruth_fn)

    gp_measurements = []  # [ts, x, z, y, qx, qy, qz, qw] orientation is used only to initialize the vio
    step = int(args.gt_freq_hz / args.gp_freq_hz)
    for i in range(0, gt.shape[0], step):
        ts = gt[i, 0]
        p = gt[i, 1:4]
        q = gt[i, 4:]
        p[0] += np.random.normal(0.0, args.gp_std)
        p[1] += np.random.normal(0.0, args.gp_std)
        p[2] += np.random.normal(0.0, args.gp_std)
        gp_measurements.append(np.array([
            ts, p[0], p[1], p[2], q[0], q[1], q[2], q[3]
            ]))
    gp_measurements = np.asarray(gp_measurements)


    # Input and output are ROS 2 bags (directories).
    bag_name = os.path.splitext(os.path.basename(os.path.normpath(args.bag_fn)))[0]
    out_bag_name = os.path.join(args.out_dir, bag_name + '_with_gp')
    out_bag = BagWriter(out_bag_name)

    for topic, msg, t in read_messages(args.bag_fn):
        out_bag.write(topic, msg, t)


    for i, x in enumerate(gp_measurements):
        ts = x[0]
        val = x[1:]

        msg = PoseStamped()
        msg.header.stamp = sec_to_stamp(ts)
        msg.pose.position.x = val[0]
        msg.pose.position.y = val[1]
        msg.pose.position.z = val[2]

        msg.pose.orientation.x = val[3]
        msg.pose.orientation.y = val[4]
        msg.pose.orientation.z = val[5]
        msg.pose.orientation.w = val[6]
        
        out_bag.write('/gp_data', msg, msg.header.stamp)

    out_bag.close()
    print("Bag saved to %s" % out_bag_name)

    out_txt = os.path.join(args.out_dir, 'simulated_gp_measurements.txt')
    np.savetxt(out_txt, gp_measurements, header='ts x y z', fmt='%.6f')
    print('GP measurments saved to %s' % out_txt)

