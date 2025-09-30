#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

BLUE = "\033[94m"
RESET = "\033[0m"

class RobotNoCrashNode:
    def __init__(self):
        rospy.init_node('robot_no_crash_node', anonymous=True)
        rospy.loginfo(f"{BLUE}Robot No Crash Node is initialized{RESET}")

        self.cmd_vel_topic = rospy.get_param('~stdr_cmd_vel', '/cmd_vel')
        self.lidar_topic = rospy.get_param('~lidar_topic', '/scan')
        self.robot_radius = rospy.get_param('~robot_radius', 0.2)
        self.outer_radius = rospy.get_param('~outer_radius', 0.3)
        self.inner_radius = rospy.get_param('~inner_radius', 0.5)
        self.refresh_rate = 10

        rospy.loginfo(f"{BLUE}Lidar topic: {self.lidar_topic}{RESET}")
        rospy.loginfo(f"{BLUE}Robot radius: {self.robot_radius}{RESET}")
        rospy.loginfo(f"{BLUE}Outer radius: {self.outer_radius}{RESET}")
        rospy.loginfo(f"{BLUE}Inner radius: {self.inner_radius}{RESET}")

        self.des_vel_sub = rospy.Subscriber('/des_vel', Twist, self.vel_callback)
        self.lidar_sub = rospy.Subscriber(self.lidar_topic, LaserScan, self.lidar_callback)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)

        self.latest_cmd = Twist()

    def vel_callback(self, msg):
        self.latest_cmd = msg
        self.cmd_vel_pub.publish(self.latest_cmd)


    def lidar_callback(self, scan):
        if not scan.ranges:
            return  

        min_distance = min([r for r in scan.ranges if scan.range_min < r < scan.range_max], default=float('inf'))

        num_ranges = len(scan.ranges)
        front_sector = scan.ranges[num_ranges // 4 : 3 * num_ranges // 4]
        front_obstacles = [r for r in front_sector if scan.range_min < r < scan.range_max]
        obstacle_in_front = any(r < self.outer_radius for r in front_obstacles)

        adjusted_cmd = Twist()
        adjusted_cmd.linear.x = self.latest_cmd.linear.x
        adjusted_cmd.angular.z = self.latest_cmd.angular.z

        if min_distance < self.inner_radius:
            rospy.logwarn(f"{BLUE}Obstacle in inner_radius! Backing away.{RESET}")
            adjusted_cmd.linear.x = -0.7 
            adjusted_cmd.angular.z = 0.5  
        elif min_distance < self.outer_radius:
            slowdown_factor = (min_distance - self.inner_radius) / (self.outer_radius - self.inner_radius)
            adjusted_cmd.linear.x = max(0.1, self.latest_cmd.linear.x * slowdown_factor) 
            adjusted_cmd.angular.z = self.latest_cmd.angular.z
            rospy.logwarn(f"{BLUE}Obstacle in outer_radius! Slowing down to {adjusted_cmd.linear.x:.2f} m/s{RESET}")

        if adjusted_cmd.linear.x <= 0.0 and obstacle_in_front:
            rospy.logwarn(f"{BLUE}Obstacle in front! Spinning in place.{RESET}")
            adjusted_cmd.angular.z = 0.8  

        self.cmd_vel_pub.publish(adjusted_cmd)

if __name__ == '__main__':
    try:
        node = RobotNoCrashNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
