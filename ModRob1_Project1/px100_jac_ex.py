"""
IMPORTS
"""
#!/usr/bin/env python3  
import rclpy # Import the ROS client library for Python 
from rclpy.node import Node # Enables the use of rclpy's Node class
from tf2_ros import TransformException # Base class to handle exceptions
from tf2_ros.buffer import Buffer # Stores known frames and offers frame graph requests
from tf2_ros.transform_listener import TransformListener # Easy way to request and receive coordinate frame transform information
from interbotix_xs_msgs.msg import JointGroupCommand # We need the joint command message to send velcoity commands to joints
from geometry_msgs.msg import Twist # Handle float64 arrays
from sensor_msgs.msg import JointState # Include joint state message
import math # Math library
from math import sin, cos, pi # Math library
import numpy as np # Numpy library

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

    # Define the `target_frame` parameter using the link names from RVIZ
    self.declare_parameter('target_frame', 'px100/ee_gripper_link')
    self.target_frame = self.get_parameter(
        'target_frame').get_parameter_value().string_value

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    """
    CREATE PUBLISHERS AND SUBSCRIBERS
    """
    # Create velocity publishers for robot joints (waist, shoulder, elbow, wrist)  
    self.publisher_vel = self.create_publisher(Twist, 'end_eff_vel', 1)

    # Create the velocity error publisher
    self.publisher_vel_err = self.create_publisher(Twist, 'vel_err', 1)

    # Create the joint angular velocity publisher 
    self.a_pub = self.create_publisher(JointGroupCommand, 'px100/commands/joint_group', 1)

    # Initialize joint position variable
    self.angles = [0.0, 0.0, 0.0, 0.0]

    # Create the subscriber that modifies the self.angles variable with the robot joint states
    self.subscription = self.create_subscription(JointState,'px100/joint_states',self.listener_callback,1)
    self.subscription # prevent unused variable warning

    """
    DEFINE THE TIME RECURRENCE FOR THE on_timer() FUNCTION AND SET THE INITIAL VALUES FOR TIME AND HOMO MATRIX
    """
    # Call on_timer function on a set interval
    timer_period = 0.1
    self.timer = self.create_timer(timer_period, self.on_timer)
    
    # Past variables' initialization
    self.homogeneous_matrix_old = np.zeros((4, 4))
    self.homogeneous_matrix_old[3, 3] = 1.0 # Past homogeneous matrix
    self.ti = self.get_clock().now().nanoseconds / 1e9 # Initial time

  def on_timer(self):
    """
    Callback function.
    This function gets called at the specific time interval.
    """

    """
    Obtain the transformation and real velocity from ROS2
    """
    # Define the target (end effector frame) and reference frames (world frame)
    from_frame_rel = self.target_frame
    to_frame_rel = 'world'

    # Obtain the transformation matrix from ROS2
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

    # ROS2 gives us the transform in quaternion form. We want it in Rotation Matrix form!
    homogeneous_matrix = self.quaternion_to_rotation_matrix(trans.transform.rotation.x,
                                                            trans.transform.rotation.y,
                                                            trans.transform.rotation.z,
                                                            trans.transform.rotation.w)   
    # Now also include the linear part into the transformation matrix
    homogeneous_matrix[0,3] = trans.transform.translation.x
    homogeneous_matrix[1,3] = trans.transform.translation.y
    homogeneous_matrix[2,3] = trans.transform.translation.z
    
    # Obtain the transformation matrix for velocities through numerical differentiation for a delta_time of 0.1 seconds
    homogeneous_matrix_deriv = (homogeneous_matrix - self.homogeneous_matrix_old) / 0.1 # Transformation derivative
    self.homogeneous_matrix_old = homogeneous_matrix # Update your old records
    homogeneous_matrix_inv = np.linalg.inv(homogeneous_matrix)
    
    # Compute the matrix form of the twist (Nu S bracket), and obtain the translational and rotational velocities from it
    vel_brack = homogeneous_matrix_deriv @ homogeneous_matrix_inv
    ang_vel_skew_symm = vel_brack[:3, :3] 
    ang_vel = np.array([ang_vel_skew_symm[2, 1], ang_vel_skew_symm[0, 2], ang_vel_skew_symm[1, 0]]) # Angular velocity vector of gripper w.r.t world frame
    trans_vel = vel_brack[:3, 3] # Translational velocity vector of a point on the origin of the {s} frame expressed w.r.t world frame
    
    # Populate and publish the end effector velocity.
    vel_msg = Twist()
    vel_msg.linear.x = trans_vel[0]
    vel_msg.linear.y = trans_vel[1]
    vel_msg.linear.z = trans_vel[2]
    vel_msg.angular.x = ang_vel[0]
    vel_msg.angular.y = ang_vel[1]
    vel_msg.angular.z = ang_vel[2]
    self.publisher_vel.publish(vel_msg)

    """
    MAKE THE ROBOT DANCE TO YOUR TUNE!
    """
    # Obtain the time stamp of the transformation from ROS2
    t = trans.header.stamp.sec + trans.header.stamp.nanosec/1e9 # Time stamp in seconds
    
    # Create the message for publishing the velocity commands
    a_msg = JointGroupCommand()
    a_msg.name = 'arm'
    
    # Define the tempo of the bopping. The higher this value, the shorter the period of the sine functions to simulate head bopping  
    tempo = 2.2
    
    if t - self.ti <= 1:
      # Define the velocities (rad/sec) for each joint between 0 and 1 seconds. Just setting an initial position. 
      a_msg.cmd = [-0.8, 0.7, -1.0, 0.0]
    
    elif t - self.ti > 1 and t - self.ti <= 10:
      # Define the velocities (rad/sec) for each joint between 1 and 10 seconds. Bop it out!
      a_msg.cmd = [0.25, -sin((tempo*pi)*(self.get_clock().now().nanoseconds / 1e9)), sin((tempo*pi)*(self.get_clock().now().nanoseconds / 1e9)), sin((tempo*pi)*(self.get_clock().now().nanoseconds / 1e9))] # Initial velocity (rad/sec.)
    
    elif t - self.ti > 10 and t - self.ti <= 20:
      # Define the velocities (rad/sec) for each joint between 10 and 20 seconds. Bop it out!
      a_msg.cmd = [-0.20, -sin((tempo*pi)*(self.get_clock().now().nanoseconds / 1e9)), sin((tempo*pi)*(self.get_clock().now().nanoseconds / 1e9)), sin((tempo*pi)*(self.get_clock().now().nanoseconds / 1e9))] # Initial velocity (rad/sec.)
    
    elif t - self.ti > 20 and t - self.ti <= 21:
      # Define final velocities to finish the dance
      a_msg.cmd = [0.7, -0.5, 0.8, -0.2] # Initial velocity (rad/sec.)
    
    else:
      # Stop it dead in its tracks
      a_msg.cmd = [0.0, 0.0, 0.0, 0.0]

    # Publish velocity commands
    self.a_pub.publish(a_msg)

    """
    Monitor the joint states, obtain the theoretical velocity with the Jacobian, and the error between theoretical and real velocities!
    """
    # Compute twist using jacobian
    self.J = np.array([[0.0,       -np.sin(self.angles[0]),                                                            0.0,                                 0.0],
                       [0.0,        np.cos(self.angles[0]),                                                            1.0,                                 1.0],
                       [1.0,                           0.0,                                                            0.0,                                 0.0],
                       [0.0, -0.08945*np.cos(self.angles[0]), 0.035*np.sin(self.angles[1]) - 0.1*np.cos(self.angles[1]) - 0.08945, 0.1*np.sin(self.angles[2]) - 0.18945],
                       [0.0, -0.08945*np.sin(self.angles[0]),                                                            0.0,                                 0.0],
                       [0.0,                           0.0,         0.035*np.cos(self.angles[1]) + 0.1*np.sin(self.angles[1]),     0.1*np.cos(self.angles[2]) + 0.035]])
    
    # Obtain the velocity from the Jacobian
    vel_from_jac = self.J @ np.array([[a_msg.cmd[0]],
                                      [a_msg.cmd[1]],
                                      [a_msg.cmd[2]],
                                      [a_msg.cmd[3]]])
    
    # Publish the velocity error message
    vel_err_msg = Twist()
    vel_err_msg.linear.x = trans_vel[0] - vel_from_jac[3][0] #[0] required for it to be a float and not an array!
    vel_err_msg.linear.y = trans_vel[1] - vel_from_jac[4][0]
    vel_err_msg.linear.z = trans_vel[2] - vel_from_jac[5][0]
    vel_err_msg.angular.x = ang_vel[0] - vel_from_jac[0][0]
    vel_err_msg.angular.y = ang_vel[1] - vel_from_jac[1][0]
    vel_err_msg.angular.z = ang_vel[2] - vel_from_jac[2][0]
    self.publisher_vel_err.publish(vel_err_msg) 

  def quaternion_to_rotation_matrix(self, q0, q1, q2, q3):
    """
    Convert a quaternion into a rotation matrix
    """
   
    # Calculate the rotation matrix
    rotation_matrix = np.array([[q0*q0 + q1*q1 - q2*q2 - q3*q3, 2*(q1*q2 - q0*q3), 2*(q0*q2 + q1*q3)],
                                [2*(q0*q3 + q1*q2), q0*q0 - q1*q1 + q2*q2 - q3*q3, 2*(q2*q3 - q0*q1)],
                                [2*(q1*q3 - q0*q2), 2*(q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3]])

    # Create a 4x4 homogeneous transformation matrix with zero translational elements
    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = rotation_matrix

    return homogeneous_matrix
  
  # Create the listener callback for the subscriber
  def listener_callback(self, data):
      """
      Callback function.
      """
      # Display the message on the console
      self.angles = [data.position[0], data.position[1], data.position[2], data.position[3]]

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