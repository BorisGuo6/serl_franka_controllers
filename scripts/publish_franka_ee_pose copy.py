#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg


def _parse_candidates(value):
    if isinstance(value, list):
        return [str(v).strip() for v in value if str(v).strip()]
    if isinstance(value, str):
        if not value.strip():
            return []
        return [v.strip() for v in value.split(",") if v.strip()]
    return []


def _resolve_ee_frame(tf_buffer, base_frame, candidates, timeout):
    for frame in candidates:
        try:
            if tf_buffer.can_transform(base_frame, frame, rospy.Time(0), timeout):
                return frame
        except tf2_ros.TransformException:
            continue
    return None


def publish_ee_pose():
    rospy.init_node("ee_pose_publisher", anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

    base_frame = rospy.get_param("~base_frame", "panda_link0")
    ee_frame_param = rospy.get_param("~ee_frame", "")
    ee_candidates = _parse_candidates(
        rospy.get_param(
            "~ee_frame_candidates",
            ["panda_link8", "panda_hand", "panda_EE", "robotiq_arg2f_base_link"],
        )
    )
    output_topic = rospy.get_param("~output_topic", "end_effector_pose")
    publish_rate = float(rospy.get_param("~publish_rate", 10.0))
    timeout = rospy.Duration(float(rospy.get_param("~timeout", 0.2)))
    log_pose = bool(rospy.get_param("~log_pose", False))
    log_throttle = float(rospy.get_param("~log_throttle", 1.0))

    pose_pub = rospy.Publisher(output_topic, geometry_msgs.msg.PoseStamped, queue_size=10)
    rate = rospy.Rate(publish_rate)

    ee_frame = ee_frame_param if ee_frame_param else None

    while not rospy.is_shutdown():
        if not ee_frame:
            ee_frame = _resolve_ee_frame(tf_buffer, base_frame, ee_candidates, timeout)
            if not ee_frame:
                rospy.logwarn_throttle(
                    5.0,
                    "No TF for base '%s' to any of %s",
                    base_frame,
                    ee_candidates,
                )
                rate.sleep()
                continue
            rospy.loginfo("Using end-effector frame: %s", ee_frame)

        try:
            trans = tf_buffer.lookup_transform(base_frame, ee_frame, rospy.Time(0), timeout)

            pose_msg = geometry_msgs.msg.PoseStamped()
            pose_msg.header.stamp = trans.header.stamp
            pose_msg.header.frame_id = base_frame

            pose_msg.pose.position.x = trans.transform.translation.x
            pose_msg.pose.position.y = trans.transform.translation.y
            pose_msg.pose.position.z = trans.transform.translation.z
            pose_msg.pose.orientation = trans.transform.rotation

            pose_pub.publish(pose_msg)

            if log_pose:
                pos = pose_msg.pose.position
                ori = pose_msg.pose.orientation
                rospy.loginfo_throttle(
                    log_throttle,
                    "EE pose %s in %s: pos[%.4f, %.4f, %.4f] quat[%.4f, %.4f, %.4f, %.4f]",
                    ee_frame,
                    base_frame,
                    pos.x,
                    pos.y,
                    pos.z,
                    ori.x,
                    ori.y,
                    ori.z,
                    ori.w,
                )

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass

        rate.sleep()

if __name__ == '__main__':
    publish_ee_pose()
