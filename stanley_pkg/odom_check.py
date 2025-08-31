#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
# TODO CHECK: include needed ROS msg type headers and libraries

class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        # TODO: create ROS subscribers and publishers
        self.odom_subscriber=self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        x=msg.pose.pose.position.x
        y=msg.pose.pose.position.y
        qx=msg.pose.pose.orientation.y
        qy=msg.pose.pose.orientation.y
        vx=msg.twist.twist.linear.x
        vy=msg.twist.twist.linear.y
        self.get_logger().info(f'Position: ({x:.2f}, {y:.2f}) \nOrientation=({qx:.2f},{qy:.2f}) \nvel=({vx:.2f}, {vy:.2f})')

def main(args=None):
    rclpy.init(args=args)
    print("odom_check Initialized")
    odom_listner_node = OdomListener()
    rclpy.spin(odom_listner_node)

    odom_listner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
