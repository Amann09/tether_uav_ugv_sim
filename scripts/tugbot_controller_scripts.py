#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import time
# import algorithm-1
# way = algorithm-1.waypoints

global x0 
global y0

way = [(0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6), (0, 7), (1, 7), (1, 8), (2, 8), (3, 8), (4, 9), (4, 10), (4, 11), (4, 12), (3, 12), 
(2, 11), (3, 11), (3, 10), (3, 9), (2, 9), (2, 10), (1, 11), (1, 10), (1, 9), (0, 8), (0, 9), (0, 10), (0, 11), (0, 12), (0, 13), (1, 13), (1, 12), 
(2, 12), (2, 13), (3, 13), (4, 13), (5, 13), (5, 14), (4, 14), (3, 14), (2, 14), (1, 14), (0, 14), (0, 15), (1, 15), (2, 15), (3, 15), (4, 15), 
(5, 15), (6, 15), (6, 14), (6, 13), (5, 12), (6, 12), (7, 12), (8, 12), (9, 12), (10, 12), (11, 12), (10, 11), (11, 11), (11, 10), (10, 10), (9, 11), 
(8, 11), (8, 10), (9, 10), (10, 9), (10, 8), (9, 8), (9, 9), (8, 9), (8, 8), (8, 7), (7, 7), (7, 8), (6, 8), (5, 8), (4, 8), (4, 7), (4, 6), (4, 5), 
(3, 5), (3, 4), (3, 3), (2, 4), (2, 5), (3, 6), (3, 7), (2, 7), (2, 6), (1, 6), (1, 5), (1, 4), (2, 3), (1, 3), (1, 2), (1, 1), (1, 0), (2, 0), (2, 1), 
(2, 2), (3, 2), (4, 3), (5, 3), (4, 4), (5, 5), (5, 6), (5, 7), (6, 7), (7, 6), (6, 6), (6, 5), (5, 4), (6, 4), (7, 4), (6, 3), (6, 2), (6, 1), (5, 1), 
(5, 2), (4, 2), (3, 1), (3, 0), (4, 0), (4, 1), (5, 0), (6, 0), (7, 0), (8, 0), (9, 0), (10, 0), (11, 0), (12, 0), (13, 0), (14, 0), (15, 0), (15, 1), 
(15, 2), (15, 3), (15, 4), (15, 5), (15, 6), (15, 7), (15, 8), (15, 9), (15, 10), (15, 11), (15, 12), (15, 13), (14, 13), (13, 13), (12, 13), (12, 14), 
(13, 14), (14, 14), (15, 14), (15, 15), (14, 15), (13, 15), (12, 15), (11, 15), (10, 15), (11, 14), (10, 14), (9, 14), (8, 14), (9, 15), (8, 15), (7, 15),
(7, 14), (7, 13), (8, 13), (9, 13), (10, 13), (11, 13), (12, 12), (12, 11), (12, 10), (13, 11), (13, 12), (14, 12), (14, 11), (13, 10), (14, 10), (14, 9), 
(14, 8), (14, 7), (14, 6), (13, 6), (13, 5), (14, 5), (14, 4), (14, 3), (14, 2), (14, 1), (13, 1), (12, 1), (11, 1), (10, 1), (9, 1), (8, 1), (7, 1), (7, 2), 
(7, 3), (8, 2), (8, 3), (8, 4), (7, 5), (8, 6), (8, 5), (9, 5), (9, 6), (9, 7), (10, 7), (10, 6), (10, 5), (11, 6), (12, 6), (12, 5), (12, 4), (13, 4), (13, 3),
(12, 3), (13, 2), (12, 2), (11, 3), (10, 3), (11, 2), (10, 2), (9, 2), (9, 3), (9, 4), (10, 4), (11, 4), (11, 5)]


rover_way = [x for x in way if way.index(x)%2 == 0]

waypoint_reached = []

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        x0 = 0 # 3.90, 3.82
        y0 = 0 # 2.00, 2.09

        self.waypoints = [ (x - x0, y - y0) for x, y in rover_way]
        self.current_waypoint_index = 0
        self.goal_x, self.goal_y = self.waypoints[self.current_waypoint_index]

        self.get_logger().info('Tugbot controller has started...')

        self.subscriber = self.create_subscription(Odometry, '/model/tugbot/odometry', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/model/tugbot/cmd_vel', 10)

        self.kp_linear = 1.0
        self.ki_linear = 0.0
        self.kd_linear = 0.1

        self.kp_angular = 0.50 # earlier 4.0, 0.50
        self.ki_angular = 0.0
        self.kd_angular = 0.1

        self.prev_error_linear = 0.0
        self.integral_linear = 0.0

        self.prev_error_angular = 0.0
        self.integral_angular = 0.0

    def odom_callback(self, msg: Odometry):
        current_x = msg.pose.pose.position.x            # round(msg.pose.pose.position.x, 2)
        current_y = msg.pose.pose.position.y            # round(msg.pose.pose.position.y, 2)
        current_orientation = msg.pose.pose.orientation
        # current_x = msg.pose.position.x
        # current_y = msg.pose.position.y
        # current_orientation = msg.pose.orientation

        # Debugging information to verify current position and orientation
        self.get_logger().info(f"Current Position: x={current_x}, y={current_y}")
        self.get_logger().info(f"Current Orientation: {current_orientation}")

        goal_angle = math.atan2(self.goal_y - current_y, self.goal_x - current_x)
        current_angle = self.get_yaw_from_quaternion(current_orientation)
        
        error_linear = math.sqrt((self.goal_x - current_x) ** 2 + (self.goal_y - current_y) ** 2)
        error_angular = self.normalize_angle(goal_angle - current_angle)

        # PID for angular velocity
        self.integral_angular += error_angular
        derivative_angular = error_angular - self.prev_error_angular

        angular_speed = (self.kp_angular * error_angular + 
                         self.ki_angular * self.integral_angular + 
                         self.kd_angular * derivative_angular)

        self.prev_error_angular = error_angular

        # Stop if we are close to the goal
        # if error_linear == 0.0:
        if error_linear < 0.01:
            linear_speed = 0.0
            angular_speed = 0.0
            self.get_logger().info('Waypoint reached!')
            a = msg.pose.pose.position.x
            b = msg.pose.pose.position.y
            waypoint_reached.append((a, b))

            # Move to the next waypoint
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.goal_x, self.goal_y = self.waypoints[self.current_waypoint_index]
                self.get_logger().info(f"Moving to next waypoint: {self.goal_x}, {self.goal_y}")
            else:
                self.get_logger().info('All waypoints reached!')
                self.publisher.publish(Twist())
                exit()
                return

        # Publish velocity commands
        cmd_vel = Twist()
        if abs(angular_speed) > 0.1:
            self.get_logger().info('Approaching towards Goal Angle..')
            linear_speed = 0.0
            cmd_vel.linear.x = linear_speed
            cmd_vel.angular.z = angular_speed
            self.get_logger().info(f"ANGULAR SPEED: {str(angular_speed)}")
            self.publisher.publish(cmd_vel)
        else:
            self.get_logger().info('Approaching towards Goal Position..')

            # PID for linear velocity
            self.integral_linear += error_linear
            derivative_linear = error_linear - self.prev_error_linear

            linear_speed = (self.kp_linear * error_linear + 
                            self.ki_linear * self.integral_linear + 
                            self.kd_linear * derivative_linear)
            
            if linear_speed < 0.01:
                linear_speed += 1.0
            elif linear_speed < 1.0 and linear_speed > 0.01:
                linear_speed += 0.5

            cmd_vel.linear.x = linear_speed
            self.get_logger().info(f"LINEAR SPEED: {str(linear_speed)}")
            cmd_vel.angular.z = 0.0
            self.publisher.publish(cmd_vel)

            self.prev_error_linear = error_linear

    def get_yaw_from_quaternion(self, orientation):
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        siny_cosp = +2.0 * (w * z + x * y)
        cosy_cosp = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        if angle > math.pi:
            angle -= 2.0 * math.pi
        elif angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)

