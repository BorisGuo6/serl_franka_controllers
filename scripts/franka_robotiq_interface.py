#!/usr/bin/env python3
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input as InputMsg
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output as OutputMsg
from std_msgs.msg import Float64MultiArray, Int32MultiArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def _clamp(value, low=0, high=255):
    try:
        value = int(value)
    except (TypeError, ValueError):
        return low
    return max(low, min(high, value))


class FrankaRobotiqInterface:
    def __init__(self):
        self.base_frame = rospy.get_param("~base_frame", "panda_link0")
        self.command_topic = rospy.get_param("~command_topic", "command_7d_pose")
        self.command_out_topic = rospy.get_param(
            "~command_out_topic",
            "/cartesian_impedance_controller/equilibrium_pose",
        )
        self.ee_pose_topic = rospy.get_param("~ee_pose_topic", "/end_effector_pose")
        self.ee_pose_7d_topic = rospy.get_param("~ee_pose_7d_topic", "end_effector_pose_7d")
        self.trajectory_topic = rospy.get_param("~trajectory_topic", "command_7d_pose_array")
        self.trajectory_rate = float(rospy.get_param("~trajectory_rate", 10.0))
        self.euler_axes = rospy.get_param("~euler_axes", "rxyz")
        self.log_throttle = float(rospy.get_param("~log_throttle", 2.0))

        self.gripper_input_topic = rospy.get_param(
            "~gripper_input_topic",
            "Robotiq2FGripperRobotInput",
        )
        self.gripper_output_topic = rospy.get_param(
            "~gripper_output_topic",
            "Robotiq2FGripperRobotOutput",
        )
        self.gripper_command_topic = rospy.get_param(
            "~gripper_command_topic",
            "robotiq_gripper/command",
        )
        self.gripper_state_topic = rospy.get_param(
            "~gripper_state_topic",
            "robotiq_gripper/state",
        )
        self.gripper_state_index = int(rospy.get_param("~gripper_state_index", 0))
        self.default_gripper_pose = _clamp(rospy.get_param("~default_gripper_pose", 0))
        self.default_gripper_speed = _clamp(rospy.get_param("~default_gripper_speed", 200))
        self.default_gripper_force = _clamp(rospy.get_param("~default_gripper_force", 50))
        self.auto_activate = bool(rospy.get_param("~auto_activate", True))

        if self.trajectory_rate <= 0.0:
            rospy.logwarn("trajectory_rate must be > 0.0, using 10.0")
            self.trajectory_rate = 10.0

        self._last_pose = None
        self._last_gripper_pose = None

        self._pose_pub = rospy.Publisher(
            self.command_out_topic,
            PoseStamped,
            queue_size=1,
        )
        self._gripper_pub = rospy.Publisher(
            self.gripper_output_topic,
            OutputMsg,
            queue_size=1,
        )
        self._gripper_state_pub = rospy.Publisher(
            self.gripper_state_topic,
            Int32MultiArray,
            queue_size=10,
        )
        self._pose7_pub = rospy.Publisher(
            self.ee_pose_7d_topic,
            Float64MultiArray,
            queue_size=1,
        )

        self._command_sub = rospy.Subscriber(
            self.command_topic,
            Float64MultiArray,
            self._command_cb,
            queue_size=1,
        )
        self._trajectory_sub = rospy.Subscriber(
            self.trajectory_topic,
            Float64MultiArray,
            self._trajectory_cb,
            queue_size=1,
        )
        self._ee_pose_sub = rospy.Subscriber(
            self.ee_pose_topic,
            PoseStamped,
            self._ee_pose_cb,
            queue_size=1,
        )
        self._gripper_state_sub = rospy.Subscriber(
            self.gripper_input_topic,
            InputMsg,
            self._gripper_state_cb,
            queue_size=1,
        )
        self._gripper_cmd_sub = rospy.Subscriber(
            self.gripper_command_topic,
            Int32MultiArray,
            self._gripper_command_cb,
            queue_size=1,
        )

        self._trajectory_lock = threading.Lock()
        self._trajectory_cancel = threading.Event()
        self._trajectory_thread = None

        if self.auto_activate:
            rospy.sleep(0.2)
            self._activate_gripper()

        rospy.loginfo(
            "Franka-robotiq interface: command %s -> %s, trajectory %s, gripper %s/%s, pose7 %s",
            self.command_topic,
            self.command_out_topic,
            self.trajectory_topic,
            self.gripper_output_topic,
            self.gripper_input_topic,
            self.ee_pose_7d_topic,
        )

    def _command_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 7:
            rospy.logwarn_throttle(
                self.log_throttle,
                "Expected 7 values [x y z roll pitch yaw gripper], got %d",
                len(msg.data),
            )
            return

        x, y, z, roll, pitch, yaw, gripper = msg.data[:7]
        self._publish_pose(x, y, z, roll, pitch, yaw, self.base_frame)

        self._publish_gripper_command(gripper)

    def _trajectory_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 7 or len(msg.data) % 7 != 0:
            rospy.logwarn_throttle(
                self.log_throttle,
                "Expected N*7 values [x y z roll pitch yaw gripper], got %d",
                len(msg.data),
            )
            return

        steps = [msg.data[i:i + 7] for i in range(0, len(msg.data), 7)]

        with self._trajectory_lock:
            if self._trajectory_thread and self._trajectory_thread.is_alive():
                self._trajectory_cancel.set()

            self._trajectory_cancel = threading.Event()
            self._trajectory_thread = threading.Thread(
                target=self._run_trajectory,
                args=(steps, self._trajectory_cancel),
                daemon=True,
            )
            self._trajectory_thread.start()

    def _run_trajectory(self, steps, cancel_event) -> None:
        rate = rospy.Rate(self.trajectory_rate)
        for step in steps:
            if rospy.is_shutdown() or cancel_event.is_set():
                return
            x, y, z, roll, pitch, yaw, gripper = step
            self._publish_pose(x, y, z, roll, pitch, yaw, self.base_frame)
            self._publish_gripper_command(gripper)
            rate.sleep()

    def _publish_gripper_command(self, gripper) -> None:
        cmd = OutputMsg()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rPR = _clamp(gripper)
        cmd.rSP = self.default_gripper_speed
        cmd.rFR = self.default_gripper_force
        self._gripper_pub.publish(cmd)

    def _activate_gripper(self) -> None:
        cmd = OutputMsg()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rSP = self.default_gripper_speed
        cmd.rFR = self.default_gripper_force
        self._gripper_pub.publish(cmd)

    def _gripper_command_cb(self, msg: Int32MultiArray) -> None:
        if not msg.data:
            rospy.logwarn("Robotiq command ignored: empty message")
            return
        position = _clamp(msg.data[0])
        speed = _clamp(msg.data[1]) if len(msg.data) > 1 else self.default_gripper_speed
        force = _clamp(msg.data[2]) if len(msg.data) > 2 else self.default_gripper_force

        cmd = OutputMsg()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rPR = position
        cmd.rSP = speed
        cmd.rFR = force
        self._gripper_pub.publish(cmd)

    def _publish_pose(self, x, y, z, roll, pitch, yaw, frame_id) -> None:
        quat = quaternion_from_euler(roll, pitch, yaw, axes=self.euler_axes)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        self._pose_pub.publish(pose_msg)

    def _ee_pose_cb(self, msg: PoseStamped) -> None:
        quat = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        roll, pitch, yaw = euler_from_quaternion(quat, axes=self.euler_axes)
        self._last_pose = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            roll,
            pitch,
            yaw,
        )
        self._publish_pose7()

    def _gripper_state_cb(self, msg: InputMsg) -> None:
        state = Int32MultiArray()
        state.data = [msg.gPO, msg.gPR, msg.gCU, msg.gSTA, msg.gOBJ, msg.gFLT]
        self._gripper_state_pub.publish(state)

        if self.gripper_state_index >= len(state.data):
            rospy.logwarn_throttle(
                self.log_throttle,
                "Robotiq state index %d out of range (len=%d)",
                self.gripper_state_index,
                len(state.data),
            )
            return
        self._last_gripper_pose = _clamp(state.data[self.gripper_state_index])
        self._publish_pose7()

    def _publish_pose7(self) -> None:
        if self._last_pose is None:
            return
        gripper = (
            self._last_gripper_pose
            if self._last_gripper_pose is not None
            else self.default_gripper_pose
        )
        out_msg = Float64MultiArray()
        out_msg.data = [
            self._last_pose[0],
            self._last_pose[1],
            self._last_pose[2],
            self._last_pose[3],
            self._last_pose[4],
            self._last_pose[5],
            float(gripper),
        ]
        self._pose7_pub.publish(out_msg)


def main() -> None:
    rospy.init_node("franka_robotiq_interface", anonymous=True)
    FrankaRobotiqInterface()
    rospy.spin()


if __name__ == "__main__":
    main()
