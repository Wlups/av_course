#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TurtlebotMappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")
        self.get_logger().info("Mapping Node has started.")

        # Publisher to send movement commands
        self._pose_publisher = self.create_publisher(
            Twist, "/cmd_vel", 10
        )

        # Subscriber to receive LiDAR data
        self._scan_listener = self.create_subscription(
            LaserScan, "/scan", self.robot_controller, 10
        )

    def robot_controller(self, scan: LaserScan):
        cmd = Twist()

        # Define obstacle detection thresholds
        front_range = 1.3   # Increased front obstacle detection distance threshold (previously 0.5)
        side_range = 1.3    # Increased side obstacle detection distance threshold (previously 0.7)
        safe_distance = 1.0 # Safe distance for forward movement
        turn_speed = 0.7    # Turning speed
        forward_speed = 0.5 # Slower forward speed to avoid collisions
        reverse_speed = 0.1 # Reverse speed for tight spaces

        # Divide LiDAR scan into sections for better obstacle detection
        a = 15  # Angle range for obstacle detection

        # Extract directional distances
        front_dist = min(scan.ranges[:a+1] + scan.ranges[-a:])
        left_dist = min(scan.ranges[90-a:90+a+1])
        right_dist = min(scan.ranges[270-a:270+a+1])

        # Add a safety margin around the robot (don't go too close to walls)
        safety_margin = 0.1  # 10 cm safety margin from walls

        # Handle obstacle detection and movement decisions
        if front_dist < front_range + safety_margin:  # Obstacle ahead
            if left_dist > right_dist:
                cmd.linear.x = 0.05  # Move slowly forward
                cmd.angular.z = turn_speed  # Turn left slowly to avoid collision
            else:
                cmd.linear.x = 0.05  # Move slowly forward
                cmd.angular.z = -turn_speed  # Turn right slowly to avoid collision
        elif left_dist < side_range + safety_margin:  # Obstacle on the left
            cmd.linear.x = forward_speed
            cmd.angular.z = turn_speed  # Turn right to avoid obstacle
        elif right_dist < side_range + safety_margin:  # Obstacle on the right
            cmd.linear.x = forward_speed
            cmd.angular.z = -turn_speed  # Turn left to avoid obstacle
        else:
            # No immediate obstacles, move forward
            cmd.linear.x = forward_speed
            cmd.angular.z = 0.0

        # Handle 180-degree turn at a corner with more caution
        if front_dist < front_range + safety_margin and left_dist > side_range and right_dist > side_range:
            # If there is space on both sides, slowly execute a 180-degree turn
            cmd.linear.x = 0.05  # Slow down to make the turn safely
            cmd.angular.z = 1.0  # Gradual turn

        # Add backward logic if both sides and front are blocked (tight corner)
        if front_dist < front_range and left_dist < side_range and right_dist < side_range:
            cmd.linear.x = -reverse_speed  # Move backward if surrounded
            cmd.angular.z = 0.0  # No turning, just reverse to find more space

        # Publish the command to move the robot
        self._pose_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotMappingNode()
    rclpy.spin(node)
    rclpy.shutdown()
