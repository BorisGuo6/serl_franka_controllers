#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

def send_franka_pose():
    # 1. Initialize the ROS node
    rospy.init_node('franka_pose_publisher', anonymous=True)
    
    # 2. Create the publisher for the equilibrium pose
    # The topic name must match the one defined in the controller
    topic_name = '/cartesian_impedance_controller/equilibrium_pose'
    pub = rospy.Publisher(topic_name, PoseStamped, queue_size=10)
    
    # Wait for the publisher to connect to the controller
    rospy.loginfo(f"Waiting for controller on {topic_name}...")
    rospy.sleep(1.0)

    # 3. Define your target pose
    # Position: x, y, z in meters
    target_pos = [0.4392239069582396, -0.013152421632842498, 0.5628677699331703]
    target_euler = [np.pi, 0, -np.pi/4]

    if target_euler:
        target_quat = R.from_euler('xyz', target_euler).as_quat()
    else:
        target_quat = [-0.930686537968667, 0.36358312993957986, -0.02889920126335501, -0.028190455978357287]
        target_euler = R.from_quat(target_quat).as_euler('xyz')

    # 4. Construct the PoseStamped message
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "panda_link0" # Ensure this matches your robot's base frame
    
    msg.pose.position.x = target_pos[0]
    msg.pose.position.y = target_pos[1]
    msg.pose.position.z = target_pos[2]
    
    msg.pose.orientation.x = target_quat[0]
    msg.pose.orientation.y = target_quat[1]
    msg.pose.orientation.z = target_quat[2]
    msg.pose.orientation.w = target_quat[3]

    # 5. Publish the message
    rospy.loginfo(f"Sending pose: Pos={target_pos}, Euler={target_euler}, quat={target_quat}")
    pub.publish(msg)
    
    # The controller uses an internal filter, so we give it a moment to reach the target
    rospy.sleep(1.0)
    rospy.loginfo("Command complete.")

if __name__ == '__main__':
    try:
        send_franka_pose()
    except rospy.ROSInterruptException:
        pass
