# Copyright 2021 InOrbit, Inc.
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
#    * Neither the name of the InOrbit, Inc. nor the names of its
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
import time
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


class SampleDataNode(Node):

    def __init__(self):
        super().__init__('SampleDataNode')
        # publishers for fake data
        self.errors_publisher = self.create_publisher(String, '/troubleshooting/errorcodes', 10)
        self.state_publisher = self.create_publisher(String, '/mode', 10)
        self.battery_publisher = self.create_publisher(BatteryState, '/battery', 10)
        self.battery_runtime_publisher = self.create_publisher(Float32, '/battery_runtime', 10)
        self.load_perc_available_publisher = self.create_publisher(
            Float32, '/load_perc_available', 10)
        self.errors = ''
        self.robot_state = 'navigating'
        self.battery_perc = 90

        self.create_timer(20, self.state_callback)
        self.create_timer(1, self.battery_callback)
        self.create_timer(7, self.set_error_callback)
        self.create_timer(3, self.send_error_callback)
        self.create_timer(3, self.battery_runtime_callback)
        self.create_timer(5, self.load_perc_available_callback)

        # transforming data from raw rosbag
        self.odom_listener = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_to_location_and_velocity_callback,
            10)
        self.location_publisher = self.create_publisher(PoseStamped, '/location', 10)
        self.velocity_publisher = self.create_publisher(TwistStamped, '/velocity', 10)

    def set_error_callback(self):
        self.errors = 'error_194,error_1'

    def send_error_callback(self):
        msg = String()
        msg.data = self.errors
        self.errors_publisher.publish(msg)
        self.get_logger().info('Publishing errors: "%s"' % msg.data)
        self.errors = ''

    def state_callback(self):
        self.robot_state = 'navigating' if self.robot_state == 'charging' else 'charging'
        msg = String()
        msg.data = self.robot_state
        self.state_publisher.publish(msg)
        self.get_logger().info('Publishing state: "%s"' % msg.data)

    def battery_callback(self):
        self.battery_perc = self.battery_perc + 1 \
            if self.robot_state == 'charging' else self.battery_perc - 1
        msg = BatteryState()
        msg.percentage = float(self.battery_perc)
        self.battery_publisher.publish(msg)
        self.get_logger().info('Publishing battery: "%s"' % msg)

    def battery_runtime_callback(self):
        msg = Float32(data=self.battery_perc / 50)  # at 90% the remaing time ~1.8 hours
        self.battery_runtime_publisher.publish(msg)
        self.get_logger().info('Publishing battery runtime: "%s"' % msg)

    def load_perc_available_callback(self):
        msg = Float32(data=77.1)
        self.load_perc_available_publisher.publish(msg)
        self.get_logger().info('Publishing load percentage available: "%s"' % msg)

    def odom_to_location_and_velocity_callback(self, msg):
        # msg is type Odometry
        pose = msg.pose.pose
        pose = PoseStamped(
            header=Header(stamp=Time(sec=int(time.time()))),
            pose=pose)
        self.location_publisher.publish(pose)
        twist = msg.twist.twist
        twist = TwistStamped(
            header=Header(stamp=Time(sec=int(time.time()))),
            twist=twist)
        self.velocity_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    sample_data_node = SampleDataNode()

    rclpy.spin(sample_data_node)

    sample_data_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
