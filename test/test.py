import sys
import rospy
import numpy as np
import geometry_msgs.msg as geom_msg
import time
import subprocess
import signal
import rosgraph
from dynamic_reconfigure.client import Client
from absl import app, flags, logging
from scipy.spatial.transform import Rotation as R
import os

FLAGS = flags.FLAGS
flags.DEFINE_string("robot_ip", None, "IP address of the robot.", required=True)
flags.DEFINE_string("load_gripper", 'false', "Whether or not to load the gripper.")


def main(_):
    impedence_controller = None
    roscore = None
    started_master = False
    try:
        # input("\033[33mPress enter to start roscore and the impedance controller.\033[0m")
        # try:
        #     roscore = subprocess.Popen('roscore')
        #     time.sleep(1)
        # except:
        #     pass

        # impedence_controller = subprocess.Popen(['roslaunch', 'serl_franka_controllers', 'impedance.launch',
        #                                         f'robot_ip:={FLAGS.robot_ip}', f'load_gripper:={FLAGS.load_gripper}'],
        #                                         stdout=subprocess.PIPE)

        eepub = rospy.Publisher('/cartesian_impedance_controller/equilibrium_pose', geom_msg.PoseStamped, queue_size=10)
        rospy.init_node('franka_control_api')
        
        # # Wait for the dynamic reconfigure service to be available
        # rospy.loginfo("Waiting for dynamic reconfigure service...")
        # service_name = '/cartesian_impedance_controller/dynamic_reconfigure_compliance_param_node/set_parameters'
        # try:
        #     rospy.wait_for_service(service_name, timeout=10)
        #     rospy.loginfo("Dynamic reconfigure service found!")
        # except rospy.ROSException:
        #     rospy.logerr("Dynamic reconfigure service not available. Make sure the controller is running.")
        #     rospy.logerr(f"Expected service: {service_name}")
        #     rospy.logerr("Trying to list available dynamic reconfigure services...")
        #     import subprocess
        #     try:
        #         result = subprocess.run(['rosservice', 'list'], capture_output=True, text=True, timeout=2)
        #         services = [s for s in result.stdout.split('\n') if 'dynamic_reconfigure' in s]
        #         if services:
        #             rospy.logerr("Found dynamic_reconfigure services:")
        #             for svc in services[:10]:  # Show first 10
        #                 rospy.logerr(f"  {svc}")
        #         else:
        #             rospy.logerr("No dynamic_reconfigure services found.")
        #     except:
        #         pass
        #     return
        
        # client = Client("/cartesian_impedance_controller/dynamic_reconfigure_compliance_param_node")


        # O_T_EE = np.array([
        #     [0.4185585379600525, -0.9019152522087097, -0.10657229274511337, 0.0],
        #     [-0.8795589208602905, -0.4317949712276459, 0.1998228132724762, 0.0],
        #     [-0.2262406349182129, 0.010099068284034729, -0.9740191102027893, 0.0],
        #     [0.4259663224220276, 0.03436925262212753, 0.5685074925422668, 1.0]
        # ])
        

        position = [0.437848269902081, 0.025796553615408335, 0.5803947671687736]

        # Define the orientation (using the matrix we discussed)
        # Or you can hardcode the quaternion from your `tf_echo` if you have it
        rot_matrix = np.array([
            [0.4185585379600525, -0.9019152522087097, -0.10657229274511337],
            [-0.8795589208602905, -0.4317949712276459, 0.1998228132724762],
            [-0.2262406349182129, 0.010099068284034729, -0.9740191102027893]
        ])
        quat = [-0.9968692275889068,0.05290089938975773,0.05773423464647683,-0.01095427221848378]#R.from_matrix(rot_matrix).as_quat()
        # Reset the arm
        msg = geom_msg.PoseStamped()
        msg.header.frame_id = "panda_link0"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position = geom_msg.Point(position[0], position[1], position[2])
        # quat = R.from_euler('xyz', [np.pi, 0, np.pi/2]).as_quat()
        msg.pose.orientation = geom_msg.Quaternion(quat[0], quat[1], quat[2], quat[3])
        input("\033[33m\nObserve the surroundings. Press enter to move the robot to the initial position.\033[0m")
        eepub.publish(msg)
        rospy.loginfo(f"Published initial pose: position=({position[0]}, {position[1]}, {position[2]})")
        rospy.sleep(1.0)

        rospy.sleep(1)
        
        # Setting the reference limiting values through ros dynamic reconfigure
        # for direction in ['x', 'y', 'z', 'neg_x', 'neg_y', 'neg_z']:
        #     client.update_configuration({"translational_clip_" + direction: 0.005})
        #     client.update_configuration({"rotational_clip_" + direction: 0.04})
        time.sleep(1)
        print("\nNew reference limiting values has been set")


        time.sleep(1)
        input("\033[33mPress enter to move the robot up with the reference limiting engaged. Notice that the arm motion should be slower this time because the maximum force is effectively limited. \033[0m")
        for i in range(10):
            msg = geom_msg.PoseStamped()
            msg.header.frame_id = "panda_link0"  # Changed from "0" to proper frame
            msg.header.stamp = rospy.Time.now()
            new_z = position[2] + i * 0.02
            msg.pose.position = geom_msg.Point(
                position[0], 
                position[1], 
                new_z
            )
            # Keep the same orientation from O_T_EE
            msg.pose.orientation = geom_msg.Quaternion(quat[0], quat[1], quat[2], quat[3])
            eepub.publish(msg)
            rospy.loginfo(f"Published pose {i+1}/10: z={new_z:.3f}")
            rospy.sleep(0.5)
        time.sleep(1)

        time.sleep(1)
        input("\033[33m\nPress enter to reset the robot arm back to the initial pose. \033[0m")
        for i in range(10):
            msg = geom_msg.PoseStamped()
            msg.header.frame_id = "panda_link0"  # Changed from "0" to proper frame
            msg.header.stamp = rospy.Time.now()
            new_z = position[2] + (9 - i) * 0.02
            msg.pose.position = geom_msg.Point(
                position[0], 
                position[1], 
                new_z
            )
            # Keep the same orientation from O_T_EE
            msg.pose.orientation = geom_msg.Quaternion(quat[0], quat[1], quat[2], quat[3])
            eepub.publish(msg)
            rospy.loginfo(f"Published pose {i+1}/10: z={new_z:.3f}")
            rospy.sleep(0.1)

        input("\033[33m\n \nPress enter to exit the test and stop the controller.\033[0m")
        return
    except KeyboardInterrupt:
        rospy.logwarn("Interrupted. Stopping the controller.")
    except Exception as exc:
        rospy.logerr("Error occurred: %s", exc)
    finally:
        if impedence_controller is not None:
            impedence_controller.send_signal(signal.SIGINT)
            try:
                impedence_controller.wait(timeout=10)
            except subprocess.TimeoutExpired:
                impedence_controller.terminate()
        if roscore is not None and started_master:
            roscore.send_signal(signal.SIGINT)
            try:
                roscore.wait(timeout=5)
            except subprocess.TimeoutExpired:
                roscore.terminate()


if __name__ == "__main__":
    app.run(main)
