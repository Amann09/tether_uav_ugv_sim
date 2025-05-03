#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.goal_x = float(5) - 3.82
        self.goal_y = float(2) - 2.09

        self.get_logger().info('Tugbot controller has started...')

        self.subscriber = self.create_subscription(Odometry, '/model/tugbot/odometry', self.pose_callback, 10)
        self.publisher = self.create_publisher(Twist, '/model/tugbot/cmd_vel', 10)

        self.kp_linear = 1.0
        self.ki_linear = 0.0
        self.kd_linear = 0.1

        self.kp_angular = 0.5 # earlier 4.0
        self.ki_angular = 0.0
        self.kd_angular = 0.1

        self.prev_error_linear = 0.0
        self.integral_linear = 0.0

        self.prev_error_angular = 0.0
        self.integral_angular = 0.0

    def pose_callback(self, msg: Odometry):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
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
        if error_linear < 0.1:
            linear_speed = 0.0
            angular_speed = 0.0
            self.get_logger().info('Goal reached!')
            self.publisher.publish(Twist())
            exit()
            return

        # Publish velocity commands
        cmd_vel = Twist()
        if abs(angular_speed) > 0.01:
            self.get_logger().info('Approaching towards Goal Angle..')
            linear_speed = 0.0
            cmd_vel.linear.x = linear_speed
            cmd_vel.angular.z = angular_speed
            self.publisher.publish(cmd_vel)
        else:
            self.get_logger().info('Approaching towards Goal Position..')

            # PID for linear velocity
            self.integral_linear += error_linear
            derivative_linear = error_linear - self.prev_error_linear

            linear_speed = (self.kp_linear * error_linear + 
                            self.ki_linear * self.integral_linear + 
                            self.kd_linear * derivative_linear)

            cmd_vel.linear.x = linear_speed
            cmd_vel.angular.z = 0.0
            self.publisher.publish(cmd_vel)

            self.prev_error_linear = error_linear

    def get_yaw_from_quaternion(self, orientation):
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
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
    main()


'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.goal_x = float(6)
        self.goal_y = float(4)

        self.subscriber = self.create_subscription(Odometry, '/model/tugbot/odometry', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/model/tugbot/cmd_vel', 10)

        self.kp_linear = 1.0
        self.ki_linear = 0.0
        self.kd_linear = 0.1

        self.kp_angular = 0.5 # earlier 4.0
        self.ki_angular = 0.0
        self.kd_angular = 0.1

        self.prev_error_linear = 0.0
        self.integral_linear = 0.0

        self.prev_error_angular = 0.0
        self.integral_angular = 0.0

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_orientation = msg.pose.pose.orientation
        # current_x = float(4)
        # current_y = float(2)
        # current_orientation = 0.0

        print(f"current_x: {round(current_x, 2)}, \
                current_y: {round(current_y, 2)}, \
                current_orientation: {current_orientation} \n ")

        goal_angle = math.atan2(self.goal_y - current_y, self.goal_x - current_x)
        # current_angle = self.get_yaw_from_quaternion(current_orientation)
        current_angle = math.radians(45)
        
        error_linear = math.sqrt((self.goal_x - current_x) ** 2 + (self.goal_y - current_y) ** 2)
        error_angular = self.normalize_angle(goal_angle - current_angle)

        # # PID for linear velocity
        # self.integral_linear += error_linear
        # derivative_linear = error_linear - self.prev_error_linear

        # linear_speed = (self.kp_linear * error_linear + 
        #                 self.ki_linear * self.integral_linear + 
        #                 self.kd_linear * derivative_linear)

        # PID for angular velocity
        self.integral_angular += error_angular
        derivative_angular = error_angular - self.prev_error_angular

        angular_speed = (self.kp_angular * error_angular + 
                         self.ki_angular * self.integral_angular + 
                         self.kd_angular * derivative_angular)

        # self.prev_error_linear = error_linear
        self.prev_error_angular = error_angular

        # # Stop if we are close to the goal
        # if error_linear < 0.05:
        #     linear_speed = 0.0
        #     angular_speed = 0.0
        #     
        #     # cmd_vel = Twist()
        #     # cmd_vel.linear.x = linear_speed
        #     # self.publisher.publish(cmd_vel)
        #     

        # Publish velocity commands
        cmd_vel = Twist()
        if abs(angular_speed) > 0.01:
            self.get_logger().info('Approaching towards Goal Angle..')
            linear_speed_ = 0.0
            cmd_vel.linear.x = linear_speed_
            cmd_vel.angular.z = angular_speed
            self.publisher.publish(cmd_vel)

        if abs(angular_speed) == 0.01:
            self.get_logger().info('Angle to Goal reached!!')

        if abs(angular_speed) < 0.01:
            self.get_logger().info('Approaching towards Goal Position..')

            angular_speed_ = 0.0

            # PID for linear velocity
            self.integral_linear += error_linear
            derivative_linear = error_linear - self.prev_error_linear

            linear_speed = (self.kp_linear * error_linear + 
                            self.ki_linear * self.integral_linear + 
                            self.kd_linear * derivative_linear)

            cmd_vel.linear.x = linear_speed
            cmd_vel.angular.z = angular_speed_
            self.publisher.publish(cmd_vel)

            self.prev_error_linear = error_linear

            if abs(linear_speed) < 0.05:
                self.get_logger().info('Goal reached!')
                self.publisher.publish(Twist())
                exit()
        

    def get_yaw_from_quaternion(self, orientation):
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
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
    main()

'''

'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        # self.declare_parameter('target_yaw', 0.0)
        # self.target_yaw = self.get_parameter('target_yaw').get_parameter_value().double_value
        self.target_yaw = math.radians(0)

        self.kp = 0.5
        self.ki = 0.0
        self.kd = 0.1

        self.current_yaw = 0.0
        self.yaw_error_integral = 0.0
        self.prev_yaw_error = 0.0

        self.odom_subscriber = self.create_subscription(Odometry, '/model/tugbot/odometry', self.odom_callback, 10)
        self.cmd_publisher = self.create_publisher(Twist, '/model/tugbot/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def control_loop(self):
        yaw_error = self.target_yaw - self.current_yaw
        self.yaw_error_integral += yaw_error
        yaw_error_derivative = yaw_error - self.prev_yaw_error

        control_signal = (self.kp * yaw_error +
                          self.ki * self.yaw_error_integral +
                          self.kd * yaw_error_derivative)

        twist = Twist()
        twist.angular.z = control_signal
        self.cmd_publisher.publish(twist)

        self.prev_yaw_error = yaw_error

        if abs(yaw_error) < 0.01:
            self.get_logger().info('Target yaw reached.')
            self.cmd_publisher.publish(Twist())  # Stop the UGV

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()

    # target_yaw = math.radians(0)  # Example: 45 degree counterclockwise rotation
    # node.target_yaw = target_yaw

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
