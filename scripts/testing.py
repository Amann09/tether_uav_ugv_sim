import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.publisher = self.create_publisher(PoseStamped, '/fmu/in/vehicle_trajectory_waypoint', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.coordinates = [(1.0, 1.0), (2.0, 2.0), (3.0, 3.0)]
        self.height = 3.0
        self.coordinate_index = 0
        self.state = 'takeoff'

    def timer_callback(self):
        if self.state == 'takeoff':
            self.takeoff()
        elif self.state == 'navigate':
            self.navigate()

    def takeoff(self):
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = self.height
        self.publisher.publish(self.target_pose)
        
        # Check if the drone has reached the desired height
        if abs(self.current_pose.pose.position.z - self.height) < 0.1:
            self.state = 'navigate'

    def navigate(self):
        if self.coordinate_index < len(self.coordinates):
            coord = self.coordinates[self.coordinate_index]
            self.target_pose.pose.position.x = coord[0]
            self.target_pose.pose.position.y = coord[1]
            self.target_pose.pose.position.z = self.height
            self.publisher.publish(self.target_pose)
            
            # Check if the drone has reached the desired position
            if abs(self.current_pose.pose.position.x - coord[0]) < 0.1 and abs(self.current_pose.pose.position.y - coord[1]) < 0.1:
                self.coordinate_index += 1
        else:
            self.state = 'hover'

    def hover(self):
        # Just keep publishing the last target_pose
        self.publisher.publish(self.target_pose)

    def pose_callback(self, msg):
        self.current_pose = msg

def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    rclpy.spin(drone_controller)
    drone_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
