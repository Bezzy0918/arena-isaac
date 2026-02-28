import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import sys
import time
import argparse

class SensorSyncChecker(Node):
    def __init__(self, lidar_topic, odom_topic, lidar_type):
        super().__init__('sensor_sync_checker')
        self.lidar_topic = lidar_topic
        self.odom_topic = odom_topic
        self.lidar_type = lidar_type
        
        self.latest_clock = None
        self.latest_odom = None
        self.latest_lidar = None
        self.sync_ok_count = 0
        self.sync_fail_count = 0
        self.max_samples = 20
        self.samples_collected = 0

        self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        
        if self.lidar_type == 'PointCloud2':
            self.create_subscription(PointCloud2, self.lidar_topic, self.lidar_callback, 10)
        else:
            self.create_subscription(LaserScan, self.lidar_topic, self.lidar_callback, 10)
        
        self.get_logger().info(f"Subscribed to /clock, {self.odom_topic}, and {self.lidar_topic} ({self.lidar_type})")

    def clock_callback(self, msg):
        self.latest_clock = msg.clock.sec + msg.clock.nanosec * 1e-9

    def odom_callback(self, msg):
        self.latest_odom = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.check_sync()

    def lidar_callback(self, msg):
        self.latest_lidar = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.check_sync()

    def check_sync(self):
        if self.latest_clock is None or self.latest_odom is None or self.latest_lidar is None:
            return

        diff_lidar_clock = abs(self.latest_lidar - self.latest_clock)
        diff_odom_clock = abs(self.latest_odom - self.latest_clock)
        
        # Threshold: 0.5 seconds
        threshold = 0.5
        
        is_sync = diff_lidar_clock < threshold and diff_odom_clock < threshold
        
        status = "OK" if is_sync else "FAIL"
        self.get_logger().info(
            f"[{status}] Clock: {self.latest_clock:.3f}, Odom: {self.latest_odom:.3f}, Lidar: {self.latest_lidar:.3f} | "
            f"Diff L-C: {diff_lidar_clock:.3f}, Diff O-C: {diff_odom_clock:.3f}"
        )
        
        if is_sync:
            self.sync_ok_count += 1
        else:
            self.sync_fail_count += 1
            
        self.samples_collected += 1
        
        if self.samples_collected >= self.max_samples:
            self.report_and_exit()

    def report_and_exit(self):
        self.get_logger().info("--- Test Summary ---")
        self.get_logger().info(f"Total Samples: {self.samples_collected}")
        self.get_logger().info(f"Sync OK: {self.sync_ok_count}")
        self.get_logger().info(f"Sync FAIL: {self.sync_fail_count}")
        
        if self.sync_ok_count > self.sync_fail_count:
            self.get_logger().info("TEST PASSED: Sensors are synchronized.")
            sys.exit(0)
        else:
            self.get_logger().error("TEST FAILED: Sensors are NOT synchronized.")
            sys.exit(1)

def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description='Check sensor synchronization.')
    parser.add_argument('--lidar_topic', type=str, default='/lidar', help='Lidar topic name')
    parser.add_argument('--odom_topic', type=str, default='/odom', help='Odometry topic name')
    parser.add_argument('--lidar_type', type=str, default='LaserScan', choices=['LaserScan', 'PointCloud2'], help='Lidar message type')
    
    # Filter out ROS args
    args_parsed, _ = parser.parse_known_args()
    
    checker = SensorSyncChecker(args_parsed.lidar_topic, args_parsed.odom_topic, args_parsed.lidar_type)
    try:
        rclpy.spin(checker)
    except SystemExit:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        checker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()