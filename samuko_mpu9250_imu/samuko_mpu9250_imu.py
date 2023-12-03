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

from time import sleep

from samuko_mpu9250_imu.modules.samuko_mpu9250_imu_serial_comm_lib import IMUSerialComm
from samuko_mpu9250_imu.modules.NodeParameters import NodeParameters

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu


class SamukoMPU9250(Node):
    """Provide an interface for accessing the sensor's features & data."""

    def __init__(self):
        # Initialize parent (ROS Node)
        super().__init__('samuko_mpu9250_imu')
        
        # Initialize ROS2 Node Parameters:
        self.param = NodeParameters(self)

        self.frame_id = self.param.frame_id.value
        self.serial_port = self.param.serial_port.value
        QoSProf = QoSProfile(depth=10)


        self.ser = IMUSerialComm(self.serial_port, 115200, 0.1)
        for i in range(20):
          sleep(1.0)
          self.get_logger().info('waiting for imu to configure: %d sec' %(i))

        self.get_logger().info('configuration complete')
        self.get_logger().info('publishing imu data ...')

        self.imu_data = Imu()

        roll_rate_variance = self.ser.get('rRate-var')
        pitch_rate_variance = self.ser.get('pRate-var')
        yaw_rate_variance = self.ser.get('yRate-var')

        lin_accx_variance = self.ser.get('accx-var')
        lin_accy_variance = self.ser.get('accy-var')
        lin_accz_variance = self.ser.get('accz-var')

        self.imu_data.orientation_covariance = [
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
        ]

        self.imu_data.angular_velocity_covariance = [
            roll_rate_variance, 0.0, 0.0,
            0.0, pitch_rate_variance, 0.0,
            0.0, 0.0, yaw_rate_variance
        ]

        self.imu_data.linear_acceleration_covariance = [
            lin_accx_variance, 0.0, 0.0,
            0.0, lin_accy_variance, 0.0,
            0.0, 0.0, lin_accz_variance
        ]


        # create topic publishers:
        self.pub_imu_data = self.create_publisher(Imu, self.frame_id + '/data', QoSProf)

        read_and_pub_freq = 20.0 # Hz
        read_and_pub_period = 1.0/read_and_pub_freq  # seconds
        self.timer = self.create_timer(read_and_pub_period, self.read_and_publish_imu_data)


    def read_and_publish_imu_data(self):
        """Read IMU data from the sensor, and publish."""

        # read from sensor
        qw, qx, qy, qz = self.ser.get("quat")
        roll_rate, pitch_rate, yaw_rate = self.ser.get("rpy-rate")
        accX, accY, accZ = self.ser.get("acc-cal")

        # update imu message
        self.imu_data.header.stamp = self.get_clock().now().to_msg()
        self.imu_data.header.frame_id = self.param.frame_id.value

        self.imu_data.orientation.w = qw
        self.imu_data.orientation.x = qx
        self.imu_data.orientation.y = qy
        self.imu_data.orientation.z = qz

        self.imu_data.angular_velocity.x = roll_rate
        self.imu_data.angular_velocity.y = pitch_rate
        self.imu_data.angular_velocity.z = yaw_rate

        self.imu_data.linear_acceleration.x = accX
        self.imu_data.linear_acceleration.y = accY
        self.imu_data.linear_acceleration.z = accZ

        # publish imu data msg
        self.pub_imu_data.publish(self.imu_data)


def main(args=None):
    rclpy.init(args=args)

    samuko_mpu9250_imu_node = SamukoMPU9250()

    rclpy.spin(samuko_mpu9250_imu_node)

    samuko_mpu9250_imu_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()