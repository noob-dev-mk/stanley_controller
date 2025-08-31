#!/usr/bin/env python3
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
# TODO CHECK: include needed ROS msg type headers and libraries
# this will use some hardcoded values of centerline and implement stanley

class StanleyV1(Node):
    def __init__(self):
        super().__init__('stanley_node')
        # TODO: create ROS subscribers and publishers
        self.odom_subscriber=self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        self.vel_publisher=self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        #parameters
        self.k = 1.0            
        self.softening_factor = 0.1   
        self.steer_limit = np.deg2rad(30)  #steering limit
        self.v_des = 1.0        #speed in (m/s)

    def get_yaw(self, q):
        """Convert quaternion to yaw angle (radians)."""
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)
    
    def odom_callback(self, msg):
        x=msg.pose.pose.position.x
        y=msg.pose.pose.position.y
        q=msg.pose.pose.orientation
        yaw=self.get_yaw(q)
        vx=msg.twist.twist.linear.x
        vy=msg.twist.twist.linear.y
        v=(vx**2+vy**2)**0.5
        self.get_logger().info(f"Current Position: ({x:.2f}, {y:.2f}) \nYaw=({np.rad2deg(yaw):.2f} degrees) \nCurrent Velocity=({vx:.2f}, {vy:.2f})")
        #we got the current pose data of the car now we need to find the nearest point

        centerline_xy = np.array([
            [  0.0,          0.0        ],
            [ -0.383937,    -0.10320847 ],
            [ -0.76787535,  -0.20641169 ],
            [ -1.15181402,  -0.30961126 ],
            [ -1.53575221,  -0.41280907 ],
            [ -1.91968913,  -0.51600672 ],
            [ -2.30362374,  -0.61920597 ],
            [ -2.68755533,  -0.72240863 ],
            [ -3.07148302,  -0.82561631 ],
            [ -3.45540602,  -0.92883051 ],
            [ -3.83932448,  -1.03205123 ],
            [ -4.22323865,  -1.13527824 ],
            [ -4.60714859,  -1.23851138 ],
            [ -4.99105457,  -1.34175048 ],
            [ -5.3749568,   -1.44499531 ],
            [ -5.75885553,  -1.54824555 ],
            [ -6.14275092,  -1.6515012  ],
            [ -6.52664313,  -1.75476186 ],
            [ -6.91053239,  -1.85802746 ],
            [ -7.29441895,  -1.96129775 ],
            [ -7.67830297,  -2.06457257 ],
            [ -8.06218467,  -2.16785169 ],
            [ -8.44606423,  -2.27113479 ],
            [ -8.8299418,   -2.37442186 ],
            [ -9.21381778,  -2.4777126  ],
            [ -9.59769217,  -2.58100675 ],
            [ -9.9815652,   -2.68430425 ],
            [-10.3654372,   -2.78760477 ]
        ])
        psi, e = self.nearest_point_tangent(x, y, centerline_xy)
        # self.get_logger().info(f'psi:{psi:.2f}, crosstracks_error:{e:.2f}')
        
        heading_phi=psi-yaw
        # self.get_logger().info(f'Before warping: heading_phi:{heading_phi:.2f}')
        heading_phi= (heading_phi + np.pi) % (2*np.pi) - np.pi #wrapping angles
        crosstrack_factor=np.arctan2((self.k * e), (v + self.softening_factor))
        # self.get_logger().info(f'before clipping: crosstrack_factor: {crosstrack_factor:.2f}')

        
        self.steer_angle=heading_phi+crosstrack_factor
        self.steer_angle=np.clip(self.steer_angle, -self.steer_limit, self.steer_limit)
        # self.get_logger().info(f'steer angle:{self.steer_angle:.2f}, heading_phi:{heading_phi:.2f}, crosstrack_factor: {crosstrack_factor:.2f}')

        self.publish_velocity()

    def publish_velocity(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = self.v_des
        msg.drive.steering_angle = self.steer_angle
        self.vel_publisher.publish(msg)
        self.get_logger().info(f'Publishing -> speed: {msg.drive.speed}, steering: {msg.drive.steering_angle}')

    def nearest_point_tangent(self, curr_x, curr_y, centerline_arr):
        diff_arr=centerline_arr-np.array([curr_x,curr_y])
        dist_arr=np.sum(diff_arr**2,axis=1)
        i=np.argmin(dist_arr)
        i2 = (i + 1) % len(centerline_arr)

        path_tangent=centerline_arr[i2]-centerline_arr[i]
        psi=np.arctan2(path_tangent[1], path_tangent[0]) #[1] is y2-y1 and [0] is x2-x1

        #now the with nearest pt and currentx,y
        vec_nearest_current=np.array([curr_x,curr_y])-centerline_arr[i]
        proj_path_tan= (np.dot(vec_nearest_current,path_tangent)/np.dot(path_tangent,path_tangent))*path_tangent
        #to get the direction on crosstrack error (e) we will do crossproduct
        cross=np.cross(vec_nearest_current,path_tangent)
        e= np.sign(cross) * np.linalg.norm(vec_nearest_current - proj_path_tan) #this will give crosstrack erroe with direction

        return psi,e

    
def main(args=None):
    rclpy.init(args=args)
    print("Stanley V1 Initialized")
    stanley_v1_node = StanleyV1()
    try:
        rclpy.spin(stanley_v1_node)
    except KeyboardInterrupt:
        pass
    stanley_v1_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



