"""Small helpers to read and write ROS 2 bags from offline scripts.

Replaces the ROS 1 `rosbag.Bag` API that the dataset/evaluation scripts used:

    from vikit_py.rosbag_utils import BagReader, BagWriter, read_messages, stamp_to_sec, sec_to_stamp

    for topic, msg, t in read_messages('my_bag', topics=['/gps']):
        print(topic, stamp_to_sec(msg.header.stamp), stamp_to_sec(t))

    with BagWriter('out_bag') as out:
        out.write('/gps', msg, t)

`t` is the receive time as a builtin_interfaces/Time message (ROS 1: rospy.Time).
ROS 1 bags must be converted first, e.g. `rosbags-convert --src in.bag --dst out_dir`.
"""
import math

import rosbag2_py
from builtin_interfaces.msg import Time
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_to_sec(stamp):
    """builtin_interfaces/Time (or a float) to seconds (ROS 1: stamp.to_sec())."""
    if isinstance(stamp, (int, float)):
        return float(stamp)
    return stamp.sec + stamp.nanosec * 1e-9


def sec_to_stamp(seconds):
    """Seconds to builtin_interfaces/Time (ROS 1: rospy.Time.from_sec())."""
    nanoseconds = int(round(seconds * 1e9))
    return nsec_to_stamp(nanoseconds)


def nsec_to_stamp(nanoseconds):
    sec = math.floor(nanoseconds / 1e9)
    return Time(sec=int(sec), nanosec=int(nanoseconds - sec * 1000000000))


def stamp_to_nsec(stamp):
    if isinstance(stamp, (int, float)):
        return int(round(stamp * 1e9))
    return stamp.sec * 1000000000 + stamp.nanosec


def read_messages(bag_path, topics=None):
    """Yields (topic, deserialized message, receive time as builtin_interfaces/Time)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topics:
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    msg_classes = {}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in msg_classes:
            msg_classes[topic] = get_message(type_map[topic])
        yield topic, deserialize_message(data, msg_classes[topic]), nsec_to_stamp(t_ns)


class BagReader:
    """Reads a ROS 2 bag (ROS 1: rosbag.Bag(path, 'r'))."""

    def __init__(self, bag_path):
        self._bag_path = bag_path

    def read_messages(self, topics=None):
        if isinstance(topics, str):
            topics = [topics]
        return read_messages(self._bag_path, topics)

    def close(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def _type_name(msg):
    # e.g. geometry_msgs.msg._pose_stamped -> geometry_msgs/msg/PoseStamped
    package = type(msg).__module__.split('.')[0]
    return package + '/msg/' + type(msg).__name__


class BagWriter:
    """Writes messages to a new ROS 2 bag (ROS 1: rosbag.Bag(path, 'w'))."""

    def __init__(self, bag_path, storage_id='mcap'):
        self._writer = rosbag2_py.SequentialWriter()
        self._writer.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
                          rosbag2_py.ConverterOptions('cdr', 'cdr'))
        self._topics = {}

    def write(self, topic, msg, t=None):
        """t: receive time (builtin_interfaces/Time or seconds); defaults to the header stamp."""
        if topic not in self._topics:
            metadata = rosbag2_py.TopicMetadata(
                id=len(self._topics), name=topic, type=_type_name(msg), serialization_format='cdr')
            self._writer.create_topic(metadata)
            self._topics[topic] = metadata
        if t is None:
            t = msg.header.stamp
        self._writer.write(topic, serialize_message(msg), stamp_to_nsec(t))

    def close(self):
        if self._writer is not None:
            del self._writer
            self._writer = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
