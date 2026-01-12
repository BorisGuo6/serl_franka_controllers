# Commanding the Cartesian Impedance Controller

This document explains how to send commands to the `cartesian_impedance_controller` to move the Franka Panda robot to a desired goal position and orientation.

## Overview

The `CartesianImpedanceController` subscribes to the `equilibrium_pose` topic and accepts `geometry_msgs/PoseStamped` messages. When a new pose is received, the controller smoothly moves the robot's end-effector to the target position and orientation using impedance control.

## Topic Information

- **Topic Name**: `equilibrium_pose`
- **Message Type**: `geometry_msgs/PoseStamped`
- **Full Topic Path**: The topic is relative to the controller's namespace. If the controller is running with namespace `/cartesian_impedance_controller`, the full topic will be:
  ```
  /cartesian_impedance_controller/equilibrium_pose
  ```

To find the exact topic name, use:
```bash
rostopic list | grep equilibrium_pose
```

## Message Structure

The `geometry_msgs/PoseStamped` message contains:
- `header`: Standard ROS header with `seq`, `stamp`, and `frame_id`
- `pose`: Contains `position` (x, y, z) and `orientation` (x, y, z, w quaternion)

### Position
- `x`: Position along the x-axis (meters)
- `y`: Position along the y-axis (meters)
- `z`: Position along the z-axis (meters)

### Orientation (Quaternion)
- `x`, `y`, `z`, `w`: Quaternion representation of orientation
- The quaternion should be normalized (x² + y² + z² + w² = 1)

## Method 1: Using rostopic Command Line

### Basic Example

Move the robot to a specific position with default orientation:

```bash
rostopic pub -1 /cartesian_impedance_controller/equilibrium_pose geometry_msgs/PoseStamped \
  "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: 'panda_link0'}, \
   pose: {position: {x: 0.5, y: 0.0, z: 0.4}, \
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### With Current Time

```bash
rostopic pub /cartesian_impedance_controller/equilibrium_pose geometry_msgs/PoseStamped \
  "{header: {stamp: now, frame_id: 'panda_link0'}, \
   pose: {position: {x: 0.5, y: 0.0, z: 0.4}, \
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### Example: Move to a Different Orientation

```bash
rostopic pub -1 /cartesian_impedance_controller/equilibrium_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'panda_link0'}, \
   pose: {position: {x: 0.5, y: 0.0, z: 0.4}, \
          orientation: {x: 0.707, y: 0.0, z: 0.0, w: 0.707}}}"
```

## Method 2: Python Script

Create a Python script to send pose commands:

```python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def move_to_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    """
    Move robot to a target pose.
    
    Args:
        x, y, z: Target position in meters
        qx, qy, qz, qw: Target orientation as quaternion
    """
    rospy.init_node('pose_commander', anonymous=True)
    
    # Get the topic name from parameter server or use default
    topic_name = rospy.get_param('~equilibrium_pose_topic', 
                                 '/cartesian_impedance_controller/equilibrium_pose')
    
    pub = rospy.Publisher(topic_name, PoseStamped, queue_size=10)
    
    # Wait for the publisher to connect
    rospy.sleep(0.5)
    
    # Create the message
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'panda_link0'  # Base frame
    
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    
    # Publish the message
    pub.publish(msg)
    rospy.loginfo(f"Published pose: position=({x}, {y}, {z}), orientation=({qx}, {qy}, {qz}, {qw})")
    
    rospy.sleep(0.1)  # Give time for message to be sent

if __name__ == '__main__':
    try:
        # Example: Move to position (0.5, 0.0, 0.4) with default orientation
        move_to_pose(0.5, 0.0, 0.4)
        
        # Example: Move to a different pose with rotated orientation
        rospy.sleep(2.0)  # Wait 2 seconds
        move_to_pose(0.4, 0.2, 0.3, qx=0.707, qy=0.0, qz=0.0, qw=0.707)
        
    except rospy.ROSInterruptException:
        pass
```

Save this as `send_pose_command.py` and make it executable:
```bash
chmod +x send_pose_command.py
```

Run it:
```bash
python send_pose_command.py
```

## Method 3: C++ Node

Create a C++ node to send pose commands:

```cpp
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_commander");
    ros::NodeHandle nh;
    
    std::string topic_name = "/cartesian_impedance_controller/equilibrium_pose";
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>(topic_name, 10);
    
    // Wait for publisher to connect
    ros::Duration(0.5).sleep();
    
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "panda_link0";
    
    // Set position
    msg.pose.position.x = 0.5;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.4;
    
    // Set orientation (quaternion)
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;
    
    pose_pub.publish(msg);
    ROS_INFO("Published pose command");
    
    ros::spinOnce();
    return 0;
}
```

## Important Notes

### Frame Reference
- The `frame_id` in the header should typically be `'panda_link0'` (the robot base frame)
- Make sure the frame_id matches the frame you're using for your coordinates

### Coordinate System
- The robot uses a standard right-handed coordinate system
- X: forward/backward
- Y: left/right
- Z: up/down

### Safety Considerations
1. **Workspace Limits**: Ensure the target pose is within the robot's workspace
2. **Collision Avoidance**: Check for obstacles before sending commands
3. **Smooth Motion**: The controller uses impedance control with filtering, so motion will be smooth
4. **Error Clipping**: The controller clips translational and rotational errors based on configured limits

### Controller Behavior

When a new pose is received:
1. The controller sets `position_d_target_` and `orientation_d_target_` to the new values
2. The integral error term `error_i` is reset to zero (line 278 in the controller)
3. The controller smoothly interpolates to the target using the `filter_params_` (default: 0.005)
4. The robot moves using impedance control with the configured stiffness and damping

### Dynamic Reconfiguration

You can also adjust controller parameters (stiffness, damping, etc.) using dynamic reconfigure:
```bash
rosrun rqt_reconfigure rqt_reconfigure
```

Navigate to the `cartesian_impedance_controller` namespace to adjust:
- Translational/rotational stiffness
- Translational/rotational damping
- Nullspace stiffness
- Error clipping limits
- Integral gain (Ki)

## Troubleshooting

### Robot doesn't move
1. Check if the controller is running:
   ```bash
   rostopic list | grep equilibrium_pose
   ```
2. Verify the topic name matches your controller namespace
3. Check controller status:
   ```bash
   rosservice call /controller_manager/list_controllers
   ```

### Robot moves to wrong position
1. Verify the frame_id matches your coordinate system
2. Check if coordinates are in the correct units (meters)
3. Ensure quaternion is normalized

### Robot moves too fast/slow
1. Adjust the `filter_params_` in the controller (lower = slower, smoother)
2. Adjust stiffness and damping via dynamic reconfigure
3. Check error clipping limits

## Example Workflow

1. **Start the controller**:
   ```bash
   roslaunch serl_franka_controllers impedance.launch
   ```

2. **Verify the topic exists**:
   ```bash
   rostopic echo /cartesian_impedance_controller/equilibrium_pose
   ```

3. **Send a command** (in another terminal):
   ```bash
   rostopic pub -1 /cartesian_impedance_controller/equilibrium_pose \
     geometry_msgs/PoseStamped \
     "{header: {frame_id: 'panda_link0'}, \
      pose: {position: {x: 0.5, y: 0.0, z: 0.4}, \
             orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
   ```

4. **Monitor robot movement** using RViz or joint state publisher

## Additional Resources

- ROS geometry_msgs documentation: http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
- Franka Emika documentation: https://frankaemika.github.io/docs/
- Quaternion basics: https://en.wikipedia.org/wiki/Quaternion
