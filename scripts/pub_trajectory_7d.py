#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from tf.transformations import euler_from_quaternion


def _clamp_gripper(value):
    try:
        value = int(value)
    except (TypeError, ValueError):
        return 0
    return max(0, min(255, value))


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
    gripper_min,
    gripper_max,
    gripper_cycles,
):
    steps = []
    total_points = max(1, int(loops) * int(points_per_loop))
    if total_points == 1:
        angles = [0.0]
    else:
        angles = [
            (float(i) / float(total_points - 1)) * float(loops) * 2.0 * math.pi
            for i in range(total_points)
        ]

    gripper_min = _clamp_gripper(gripper_min)
    gripper_max = _clamp_gripper(gripper_max)
    gripper_cycles = max(0.0, float(gripper_cycles))

    for idx, angle in enumerate(angles):
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

        if total_points > 1 and gripper_cycles > 0.0:
            t = float(idx) / float(total_points - 1)
            phase = (t * gripper_cycles) % 1.0
            if phase <= 0.5:
                gripper = gripper_min + (phase / 0.5) * (gripper_max - gripper_min)
            else:
                gripper = gripper_max - ((phase - 0.5) / 0.5) * (gripper_max - gripper_min)
        else:
            gripper = gripper_min

        steps.append([x, y, z, roll_t, pitch_t, yaw, float(_clamp_gripper(gripper))])

    return steps


def main() -> None:
    rospy.init_node("franka_pose_trajectory_7d_publisher", anonymous=True)
    topic_name = rospy.get_param("~trajectory_topic", "/command_7d_pose_array")

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

    gripper_min = rospy.get_param("~gripper_min", 0)
    gripper_max = rospy.get_param("~gripper_max", 255)
    gripper_cycles = float(rospy.get_param("~gripper_cycles", 1.0))

    pub = rospy.Publisher(topic_name, Float64MultiArray, queue_size=1)
    rospy.loginfo("Publishing 7D trajectory to %s", topic_name)
    rospy.sleep(1.0)

    steps = _build_trajectory(
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
        gripper_min,
        gripper_max,
        gripper_cycles,
    )

    msg = Float64MultiArray()
    msg.data = [value for step in steps for value in step]
    pub.publish(msg)
    rospy.loginfo("Published %d poses", len(steps))
    rospy.sleep(0.5)


if __name__ == "__main__":
    main()
