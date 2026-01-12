#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def _make_pose(x, y, z, roll, pitch, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    quat = quaternion_from_euler(roll, pitch, yaw, axes="rxyz")
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose


def _coerce_center(value, default):
    if isinstance(value, (list, tuple)) and len(value) == 3:
        try:
            return [float(value[0]), float(value[1]), float(value[2])]
        except (TypeError, ValueError):
            return list(default)
    return list(default)


def _read_current_pose(pose_topic, timeout, default_center, default_rpy):
    try:
        msg = rospy.wait_for_message(pose_topic, PoseStamped, timeout=timeout)
    except rospy.ROSException:
        return list(default_center), tuple(default_rpy)

    center = [
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z,
    ]
    quat = (
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w,
    )
    roll, pitch, yaw = euler_from_quaternion(quat, axes="rxyz")
    return center, (roll, pitch, yaw)


def _build_trajectory(
    center,
    radius_x,
    radius_y,
    z_amplitude,
    loops,
    points_per_loop,
    roll,
    pitch,
    yaw_offset,
    yaw_follow,
    z_cycles,
    pitch_amplitude,
    pitch_cycles,
    roll_amplitude,
    roll_cycles,
):
    poses = []
    total_points = max(1, int(loops) * int(points_per_loop))
    if total_points == 1:
        angles = [0.0]
    else:
        angles = [
            (float(i) / float(total_points - 1)) * float(loops) * 2.0 * math.pi
            for i in range(total_points)
        ]

    for angle in angles:
        x = center[0] + radius_x * math.cos(angle)
        y = center[1] + radius_y * math.sin(angle)
        z = center[2] + z_amplitude * math.sin(angle * z_cycles)

        if yaw_follow:
            dx = -radius_x * math.sin(angle)
            dy = radius_y * math.cos(angle)
            yaw = math.atan2(dy, dx) + yaw_offset
        else:
            yaw = yaw_offset

        roll_t = roll + roll_amplitude * math.sin(angle * roll_cycles)
        pitch_t = pitch + pitch_amplitude * math.sin(angle * pitch_cycles)
        poses.append(_make_pose(x, y, z, roll_t, pitch_t, yaw))

    return poses


def main() -> None:
    rospy.init_node("franka_pose_trajectory_publisher", anonymous=True)
    topic_name = rospy.get_param("~trajectory_topic", "/command_pose_array")
    frame_id = rospy.get_param("~frame_id", "panda_link0")

    default_center = [0.3892239, -0.0131524, 0.5628678]
    default_rpy = (math.pi, 0.0, 0.785398)

    center = _coerce_center(rospy.get_param("~center", default_center), default_center)
    radius_x = abs(float(rospy.get_param("~radius_x", 0.1)))
    radius_y = abs(float(rospy.get_param("~radius_y", 0.1)))
    z_amplitude = abs(float(rospy.get_param("~z_amplitude", 0.05)))
    loops = max(1, int(rospy.get_param("~loops", 1)))
    points_per_loop = max(2, int(rospy.get_param("~points_per_loop", 80)))

    roll = float(rospy.get_param("~roll", default_rpy[0]))
    pitch = float(rospy.get_param("~pitch", default_rpy[1]))
    yaw_offset = float(rospy.get_param("~yaw_offset", default_rpy[2]))
    yaw_follow = bool(rospy.get_param("~yaw_follow", False))

    use_current_pose = bool(rospy.get_param("~use_current_pose", True))
    if use_current_pose:
        pose_topic = rospy.get_param("~current_pose_topic", "/end_effector_pose")
        timeout = float(rospy.get_param("~current_pose_timeout", 1.0))
        center, (roll, pitch, yaw_offset) = _read_current_pose(
            pose_topic,
            timeout,
            center,
            (roll, pitch, yaw_offset),
        )

    z_cycles = max(0.0, float(rospy.get_param("~z_cycles", 1.0)))
    pitch_amplitude = abs(float(rospy.get_param("~pitch_amplitude", 0.0)))
    pitch_cycles = max(0.0, float(rospy.get_param("~pitch_cycles", 1.0)))
    roll_amplitude = abs(float(rospy.get_param("~roll_amplitude", 0.0)))
    roll_cycles = max(0.0, float(rospy.get_param("~roll_cycles", 1.0)))

    pub = rospy.Publisher(topic_name, PoseArray, queue_size=1)
    rospy.loginfo("Publishing PoseArray to %s", topic_name)
    rospy.sleep(1.0)

    pose_array = PoseArray()
    pose_array.header.stamp = rospy.Time.now()
    pose_array.header.frame_id = frame_id
    pose_array.poses = _build_trajectory(
        center,
        radius_x,
        radius_y,
        z_amplitude,
        loops,
        points_per_loop,
        roll,
        pitch,
        yaw_offset,
        yaw_follow,
        z_cycles,
        pitch_amplitude,
        pitch_cycles,
        roll_amplitude,
        roll_cycles,
    )

    pub.publish(pose_array)
    rospy.loginfo("Published %d poses", len(pose_array.poses))
    rospy.sleep(0.5)


if __name__ == "__main__":
    main()
