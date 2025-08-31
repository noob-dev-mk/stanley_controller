import rclpy
from rclpy.node import Node
import os
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
import csv
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


class StanleyPerformanceLogger(Node):
    def __init__(self, x_ref, y_ref, start_line_y):
        super().__init__('stanley_performance_logger')

        # Subscribe to Odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Reference path
        self.x_ref = x_ref
        self.y_ref = y_ref

        # Trajectory and error storage
        self.actual_x = []
        self.actual_y = []
        self.cte_list = []
        self.time_list = []

        # Lap detection
        self.start_line_y = start_line_y
        self.last_lap_time = None
        self.crossed = False
        self.lap_count = 0

        # Make sure reports directory exists (inside the package folder)
        pkg_path = get_package_share_directory('stanley_pkg')
        self.report_dir = "/sim_ws/src/stanley_pkg/reports"
        os.makedirs(self.report_dir, exist_ok=True)

        # Timestamp for filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # CSV logging file
        csv_path = os.path.join(self.report_dir, f'performance_log_{timestamp}.csv')
        self.csv_file = open(csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['time', 'x', 'y', 'cte', 'lap_time', 'lap_count'])
        self.start_time = self.get_clock().now().to_msg().sec + \
                          self.get_clock().now().to_msg().nanosec * 1e-9

        # PDF report file
        self.pdf_filename = os.path.join(self.report_dir, f'stanley_test_report_{timestamp}.pdf')

        self.get_logger().info(f"Stanley Performance Logger Initialized. Reports will be saved to {self.report_dir}")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Compute Cross-Track Error
        distances = np.sqrt((self.x_ref - x) ** 2 + (self.y_ref - y) ** 2)
        cte = np.min(distances)

        # Time since start
        current_time = self.get_clock().now().to_msg().sec + \
                       self.get_clock().now().to_msg().nanosec * 1e-9 - self.start_time

        # Save trajectory & error
        self.actual_x.append(x)
        self.actual_y.append(y)
        self.cte_list.append(cte)
        self.time_list.append(current_time)

        lap_time = None
        # Lap detection
        if not self.crossed and y >= self.start_line_y:
            if self.last_lap_time is not None:
                lap_time = current_time - self.last_lap_time
                self.lap_count += 1
                self.get_logger().info(f'Lap {self.lap_count} time: {lap_time:.2f} s')
            self.last_lap_time = current_time
            self.crossed = True

        if y < self.start_line_y - 0.5:
            self.crossed = False

        # Log to CSV
        self.writer.writerow([current_time, x, y, cte, lap_time if lap_time else '', self.lap_count])

    def generate_report(self):
        self.get_logger().info("Generating PDF report...")
        with PdfPages(self.pdf_filename) as pdf:
            # CTE vs Time
            plt.figure(figsize=(8, 5))
            plt.plot(self.time_list, self.cte_list, label='CTE', color='red')
            plt.xlabel('Time (s)')
            plt.ylabel('Cross-Track Error (m)')
            plt.title('CTE vs Time')
            plt.grid(True)
            plt.legend()
            pdf.savefig()
            plt.close()

            # Trajectory Comparison
            plt.figure(figsize=(8, 5))
            plt.plot(self.x_ref, self.y_ref, label='Reference Path', linestyle='--', color='blue')
            plt.plot(self.actual_x, self.actual_y, label='Actual Trajectory', color='green')
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.title('Trajectory Comparison')
            plt.grid(True)
            plt.legend()
            pdf.savefig()
            plt.close()

            # Lap Times
            plt.figure(figsize=(8, 5))
            lap_times_clean = []
            with open(self.csv_file.name, "r") as f:
                reader = csv.reader(f)
                rows = list(reader)

                if len(rows) > 1:   # has header + at least 1 data row
                    for row in rows[1:]:  # skip header
                        if row[4] != '':   # lap_time column
                            try:
                                lap_times_clean.append(float(row[4]))
                            except ValueError:
                                pass

            lap_numbers = list(range(1, len(lap_times_clean) + 1))
            if lap_times_clean:
                plt.bar(lap_numbers, lap_times_clean, color='purple')
                plt.xlabel('Lap')
                plt.ylabel('Lap Time (s)')
                plt.title('Lap Times')
                plt.grid(True)
            else:
                plt.text(0.5, 0.5, "No laps completed", ha='center', va='center')
            pdf.savefig()
            plt.close()

        self.get_logger().info(f"✅ PDF report saved at: {self.pdf_filename}")


def main(args=None):
    rclpy.init(args=args)

    # Load reference path 
    pkg_path = get_package_share_directory('stanley_pkg')
    csv_file = os.path.join(pkg_path, 'data', 'Spielberg_centerline.csv')

    data = np.loadtxt(csv_file, delimiter=',', comments='#')
    centerline_xy = data[:, 0:2]

    x_ref = centerline_xy[:, 0]
    y_ref = centerline_xy[:, 1]

    start_line_y = 0.0  # adjust according to your map

    node = StanleyPerformanceLogger(x_ref, y_ref, start_line_y)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # On Ctrl+C, stop and generate report
        node.get_logger().info("Test finished, generating report...")
        node.generate_report()
    finally:
        node.csv_file.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
