"""
IMPORTS
"""
#!/usr/bin/env python3 
 
# Import the ROS client library for Python 
import rclpy 
 
# Enables the use of rclpy's Node class
from rclpy.node import Node 
 
# Base class to handle exceptions
from tf2_ros import TransformException 
 
# Stores known frames and offers frame graph requests
from tf2_ros.buffer import Buffer
 
# Easy way to request and receive coordinate frame transform information
from tf2_ros.transform_listener import TransformListener 

# We need the joint command message to send velcoity commands to joints
from interbotix_xs_msgs.msg import JointGroupCommand
 
# Handle float64 arrays
from geometry_msgs.msg import Twist

# Include joint state message
from sensor_msgs.msg import JointState
 
# Math library
import math 
from math import sin, cos, pi

# Numpy library
import numpy as np




"""
FrameListener Node
"""
class FrameListener(Node):
  """
  Subclass of the Node class.
  The class listens to coordinate transformations and 
  publishes the end-effector velocity at a specific time interval.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('map_base_link_frame_listener')

    # Declare and acquire `target_frame` parameter
    self.declare_parameter('target_frame', 'px100/ee_gripper_link')
    self.target_frame = self.get_parameter(
        'target_frame').get_parameter_value().string_value

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    # Call on_timer function on a set interval
    timer_period = 0.1
    self.timer = self.create_timer(timer_period, self.on_timer)
        
    # Past variables' initialization
    self.homogeneous_matrix_old = np.zeros((4, 4)); self.homogeneous_matrix_old[3, 3] = 1.0 # Past homogeneous matrix
    self.ti = self.get_clock().now().nanoseconds / 1e9 # Initial time

  def on_timer(self):
    """
    Callback function.
    This function gets called at the specific time interval.
    """
    # Store frame names in variables that will be used to
    # compute transformations
    from_frame_rel = self.target_frame
    to_frame_rel = 'world'

    trans = None
    
    try:
      now = rclpy.time.Time()
      trans = self.tf_buffer.lookup_transform(
                  to_frame_rel,
                  from_frame_rel,
                  now)
    except TransformException as ex:
      self.get_logger().info(
          f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
    return




"""
MAIN FUNCTION
"""
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  frame_listener_node = FrameListener()

  # Spin the node so the callback function is called.
  # Publish any pending messages to the topics.
  try:
    rclpy.spin(frame_listener_node)
  except KeyboardInterrupt:
    pass
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()