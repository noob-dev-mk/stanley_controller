#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
# TODO CHECK: include needed ROS msg type headers and libraries

#these below comments are just for checking
#geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0))

# ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped \
# "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, drive: {steering_angle: 0.2, speed: 2.0}}"
#by default the frame is the front wheel axle in ackermann 

class VelocityTesting(Node):
    def __init__(self):
        super().__init__('vel_test')
        # TODO: create ROS subscribers and publishers
        self.vel_publisher=self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_velocity) #this will be called for 1 sec
        self.timer = self.create_timer(timer_period, self.publish_velocity2) #this will be called for another 1 sec
    # i created two publish just to test if we can send out two vel and see the movement of the f1tenth sim
    def publish_velocity(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 1.0
        msg.drive.steering_angle = 0.0
        self.vel_publisher.publish(msg)
        self.get_logger().info(f'Publishing -> speed: {msg.drive.speed}, steering: {msg.drive.steering_angle}')    
        
    def publish_velocity2(self):
        msg2 = AckermannDriveStamped()
        msg2.drive.speed = 1.0
        msg2.drive.steering_angle = 0.1
        self.vel_publisher.publish(msg2)
        self.get_logger().info(f'Publishing -> speed: {msg2.drive.speed}, steering: {msg2.drive.steering_angle}')

def main(args=None):
    rclpy.init(args=args)
    print("vel_publisher Initialized")
    vel_publisher_node = VelocityTesting()
    try:
        rclpy.spin(vel_publisher_node)
    except KeyboardInterrupt:
        pass
    vel_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()