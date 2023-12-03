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


import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class TransformTestNode(Node):
    """Provide an interface for accessing the sensor's features & data."""

    def __init__(self):
        # Initialize parent (ROS Node)
        super().__init__('tf_test_node')

        self.isBroadcasted = False

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.listen_and_broadcast_imu_tf,
            10)
        self.imu_sub  # prevent unused variable warning


    def listen_and_broadcast_imu_tf(self, imu_data):
        if not self.isBroadcasted:
            self.get_logger().info('samuko_mpu9250_imu tf_test_node tf broadcast started')
            self.isBroadcasted = True
            
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = imu_data.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = imu_data.header.frame_id

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.w = imu_data.orientation.w
        t.transform.rotation.x = imu_data.orientation.x
        t.transform.rotation.y = imu_data.orientation.y
        t.transform.rotation.z = imu_data.orientation.z

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    tf_test_node = TransformTestNode()

    rclpy.spin(tf_test_node)

    tf_test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()