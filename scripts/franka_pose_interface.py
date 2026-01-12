#!/usr/bin/env python3
import threading

import rospy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Float64MultiArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class FrankaPoseInterface:
    def __init__(self):
        self.base_frame = rospy.get_param("~base_frame", "panda_link0")
        self.command_topic = rospy.get_param("~command_topic", "command_6d_pose")
        self.command_out_topic = rospy.get_param(
            "~command_out_topic",
            "/cartesian_impedance_controller/equilibrium_pose",
        )
        self.ee_pose_topic = rospy.get_param("~ee_pose_topic", "/end_effector_pose")
        self.ee_pose_6d_topic = rospy.get_param(
            "~ee_pose_6d_topic",
            "end_effector_pose_6d",
        )
        self.trajectory_topic = rospy.get_param(
            "~trajectory_topic",
            "command_pose_array",
        )
        self.trajectory_rate = float(rospy.get_param("~trajectory_rate", 10.0))
        self.euler_axes = rospy.get_param("~euler_axes", "rxyz")
        self.log_throttle = float(rospy.get_param("~log_throttle", 2.0))

        if self.trajectory_rate <= 0.0:
            rospy.logwarn("trajectory_rate must be > 0.0, using 10.0")
            self.trajectory_rate = 10.0

        self.command_pub = rospy.Publisher(
            self.command_out_topic,
            PoseStamped,
            queue_size=1,
        )
        self.ee_pose_pub = rospy.Publisher(
            self.ee_pose_6d_topic,
            Float64MultiArray,
            queue_size=1,
        )

        self.command_sub = rospy.Subscriber(
            self.command_topic,
            Float64MultiArray,
            self._command_cb,
            queue_size=1,
        )
        self.trajectory_sub = rospy.Subscriber(
            self.trajectory_topic,
            PoseArray,
            self._trajectory_cb,
            queue_size=1,
        )
        self.ee_pose_sub = rospy.Subscriber(
            self.ee_pose_topic,
            PoseStamped,
            self._ee_pose_cb,
            queue_size=1,
        )

        self._trajectory_lock = threading.Lock()
        self._trajectory_cancel = threading.Event()
        self._trajectory_thread = None

        rospy.loginfo(
            "Franka pose interface: command %s -> %s, ee pose %s -> %s",
            self.command_topic,
            self.command_out_topic,
            self.ee_pose_topic,
            self.ee_pose_6d_topic,
        )

    def _command_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 6:
            rospy.logwarn_throttle(
                self.log_throttle,
                "Expected 6 values [x y z roll pitch yaw], got %d",
                len(msg.data),
            )
            return

        x, y, z, roll, pitch, yaw = msg.data[:6]
        quat = quaternion_from_euler(roll, pitch, yaw, axes=self.euler_axes)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self._publish_pose(pose, self.base_frame)

    def _trajectory_cb(self, msg: PoseArray) -> None:
        if not msg.poses:
            rospy.logwarn_throttle(
                self.log_throttle,
                "Received empty PoseArray on %s",
                self.trajectory_topic,
            )
            return

        frame_id = msg.header.frame_id if msg.header.frame_id else self.base_frame
        poses = list(msg.poses)

        with self._trajectory_lock:
            if self._trajectory_thread and self._trajectory_thread.is_alive():
                self._trajectory_cancel.set()

            self._trajectory_cancel = threading.Event()
            self._trajectory_thread = threading.Thread(
                target=self._run_trajectory,
                args=(poses, frame_id, self._trajectory_cancel),
                daemon=True,
            )
            self._trajectory_thread.start()

    def _run_trajectory(self, poses, frame_id, cancel_event) -> None:
        rate = rospy.Rate(self.trajectory_rate)
        for pose in poses:
            if rospy.is_shutdown() or cancel_event.is_set():
                return
            self._publish_pose(pose, frame_id)
            rate.sleep()

    def _ee_pose_cb(self, msg: PoseStamped) -> None:
        quat = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        roll, pitch, yaw = euler_from_quaternion(quat, axes=self.euler_axes)

        out_msg = Float64MultiArray()
        out_msg.data = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            roll,
            pitch,
            yaw,
        ]
        self.ee_pose_pub.publish(out_msg)

    def _publish_pose(self, pose: Pose, frame_id: str) -> None:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = frame_id
        pose_msg.pose = pose
        self.command_pub.publish(pose_msg)


def main() -> None:
    rospy.init_node("franka_pose_interface", anonymous=True)
    FrankaPoseInterface()
    rospy.spin()


if __name__ == "__main__":
    main()
