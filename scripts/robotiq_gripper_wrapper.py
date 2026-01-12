#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output as OutputMsg
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input as InputMsg


def _clamp(value, low=0, high=255):
    return max(low, min(high, int(value)))


class RobotiqGripperNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "Robotiq2FGripperRobotInput")
        self.output_topic = rospy.get_param("~output_topic", "Robotiq2FGripperRobotOutput")
        self.command_topic = rospy.get_param("~command_topic", "robotiq_gripper/command")
        self.state_topic = rospy.get_param("~state_topic", "robotiq_gripper/state")
        self.default_speed = _clamp(rospy.get_param("~default_speed", 255))
        self.default_force = _clamp(rospy.get_param("~default_force", 150))
        self.auto_activate = bool(rospy.get_param("~auto_activate", True))

        self._pub_cmd = rospy.Publisher(self.output_topic, OutputMsg, queue_size=1)
        self._pub_state = rospy.Publisher(self.state_topic, Int32MultiArray, queue_size=10)
        self._sub_state = rospy.Subscriber(self.input_topic, InputMsg, self._state_cb, queue_size=1)
        self._sub_cmd = rospy.Subscriber(self.command_topic, Int32MultiArray, self._command_cb, queue_size=1)

        if self.auto_activate:
            rospy.sleep(0.2)
            self._activate()

        rospy.loginfo("Robotiq node ready. Command: %s, State: %s", self.command_topic, self.state_topic)

    def _activate(self):
        cmd = OutputMsg()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rSP = self.default_speed
        cmd.rFR = self.default_force
        self._pub_cmd.publish(cmd)

    def _command_cb(self, msg):
        if not msg.data:
            rospy.logwarn("Robotiq command ignored: empty message")
            return

        position = _clamp(msg.data[0])
        speed = _clamp(msg.data[1]) if len(msg.data) > 1 else self.default_speed
        force = _clamp(msg.data[2]) if len(msg.data) > 2 else self.default_force

        cmd = OutputMsg()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rPR = position
        cmd.rSP = speed
        cmd.rFR = force
        self._pub_cmd.publish(cmd)

    def _state_cb(self, msg):
        state = Int32MultiArray()
        state.data = [msg.gPO, msg.gPR, msg.gCU, msg.gSTA, msg.gOBJ, msg.gFLT]
        self._pub_state.publish(state)


def main():
    rospy.init_node("robotiq_gripper_wrapper", anonymous=True)
    RobotiqGripperNode()
    rospy.spin()


if __name__ == "__main__":
    main()
