#!/usr/bin/env python3
import rclpy
from ament_index_python.packages import get_package_share_directory
import os
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
# TODO CHECK: include needed ROS msg type headers and libraries
# this will read csv file to get values of centerline and implement stanley

class StanleyFinal(Node):
    def __init__(self):
        super().__init__('stanley_final')
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
        self.v_des = 1.0        #speed in m/s

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
        #we will now read and process csv
        pkg_path = get_package_share_directory('stanley_pkg')
        csv_file = os.path.join(pkg_path, 'data', 'Spielberg_centerline.csv') 
        #NOTE: change the reference line csv file according to your map 

        data = np.loadtxt(csv_file, delimiter=',', comments='#')
        centerline_xy = data[:, 0:2] #this is to get x,y as an array for MY CSV it might differ on others depending on the structure of csv

        psi, e = self.nearest_point_tangent(x, y, centerline_xy)
        # self.get_logger().info(f'psi:{psi:.2f}, crosstracks_error:{e:.2f}')
        #these commented out lines was for troubleshooting 

        heading_phi=psi-yaw
        # self.get_logger().info(f'Before warping: heading_phi:{heading_phi:.2f}')

        heading_phi= (heading_phi + np.pi) % (2*np.pi) - np.pi #wrapping angles
        crosstrack_factor=np.arctan2((self.k * e), (v + self.softening_factor))
        # self.get_logger().info(f'before clipping: crosstrack_factor: {crosstrack_factor:.2f}')

        
        self.steer_angle=heading_phi+crosstrack_factor
        self.steer_angle=np.clip(self.steer_angle, -self.steer_limit, self.steer_limit) #clipping the final steer angle
        # self.get_logger().info(f'steer angle:{self.steer_angle:.2f}, heading_phi:{heading_phi:.2f}, crosstrack_factor: {crosstrack_factor:.2f}')

        self.publish_velocity()

    def publish_velocity(self):#this will publish the velocity from the publisher we created earlier
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
    print("Stanley Initialized")
    stanley = StanleyFinal()
    try:
        rclpy.spin(stanley)
    except KeyboardInterrupt:
        pass
    stanley.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



