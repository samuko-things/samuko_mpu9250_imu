# Copyright 2021 AUTHORS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the AUTHORS nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.



from rclpy.node import Node


class NodeParameters:
  """
  ROS2 Node Parameter Handling.

  https://index.ros.org/doc/ros2/Tutorials/Parameters/Understanding-ROS2-Parameters
  https://index.ros.org/doc/ros2/Tutorials/Using-Parameters-In-A-Class-Python/

  Start the node with parameters from yml file:
  ros2 run samuko_mpu9250_imu samuko_mpu9250_imu

  with the following arguments:
  --ros-args --params-file ~/<workspace>/src/samuko_mpu9250_imu/config/samuko_mpu9250_imu_params.yaml
  """

  def __init__(self, node: Node):
    node.get_logger().info('Initializing parameters')
    # Declare parameters of the ROS2 node and their default values:
    # if no parameters are given, use the default value

    # tf frame id
    node.declare_parameter('frame_id', value='')
    # serial port
    node.declare_parameter('serial_port', value='')
    
    # Sensor standard deviation squared (^2) defaults [x, y, z]
    # node.declare_parameter('acc_variance', value=[0.0, 0.0, 0.0])
    # node.declare_parameter('angular_rate_variance', value=[0.0, 0.0, 0.0])
    # node.declare_parameter('orientation_variance', value=[0.0, 0.0, 0.0])

    # get the parameters - requires CLI arguments '--ros-args --params-file <parameter file>'
    node.get_logger().info('Parameters set to:')

    try:
      self.frame_id = node.get_parameter('frame_id')
      node.get_logger().info('\tframe_id:\t\t"%s"' % self.frame_id.value)

      self.serial_port = node.get_parameter('serial_port')
      node.get_logger().info('\tserial_port:\t\t"%s"' % self.serial_port.value)

      # self.acc_variance = node.get_parameter('acc_variance')
      # node.get_logger().info('\tacc_variance:\t\t"%s"' % self.acc_variance.value)
      # self.angular_rate_variance = node.get_parameter('angular_rate_variance')
      # node.get_logger().info('\tangular_rate_variance:\t"%s"' % self.angular_rate_variance.value)
      # self.orientation_variance = node.get_parameter('orientation_variance')
      # node.get_logger().info('\torientation_variance:\t"%s"' % self.orientation_variance.value)

    except Exception as e:  # noqa: B902
      node.get_logger().warn('Could not get parameters...setting variables to default')
      node.get_logger().warn('Error: "%s"' % e)
