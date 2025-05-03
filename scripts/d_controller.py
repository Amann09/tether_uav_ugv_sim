#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleOdometry, VehicleCommandAck
import numpy as np

class OffboardControl(Node):
        """ Node for controlling a vehicle in offboard mode. """

        def __init__(self) -> None:
                super().__init__('offboard_control_takeoff_and_land')

                # Configure QoS profile for publishing and subscribing
                qos_profile = QoSProfile(reliability = ReliabilityPolicy.BEST_EFFORT, 
                                         durability = DurabilityPolicy.TRANSIENT_LOCAL,
                                         history = HistoryPolicy.KEEP_LAST, 
                                         depth = 1)
                
                # Create publishers
                self.offboard_control_mode_publisher = self.create_publisher(
                        OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
                self.trajectory_setpoint_publisher = self.create_publisher(
                        TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
                self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

                # Create subscribers
                self.vehicle_local_position_subscriber = self.create_subscription(
                        VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
                self.vehicle_status_subscriber = self.create_subscription(
                        VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
                self.vehicle_odometry_subscriber = self.create_subscription(
                        VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
                self.vehicle_command_ack_subscriber = self.create_subscription(
                        VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.vehicle_command_ack_callback, qos_profile)
                
                # Initialize variables
                self.offboard_setpoint_counter = 0
                self.vehicle_local_position = VehicleLocalPosition()
                self.vehicle_status = VehicleStatus()
                self.vehicle_odometry = VehicleOdometry()
                self.vehicle_command_ack = VehicleCommandAck()

                self.vehicle_vel_x = self.vehicle_odometry.velocity

                self.takeoff_height = -1.0

                #Create a timer to publish control commands
                self.timer = self.create_timer(0.1, self.timer_callback)

                self.taken_off = None
                self.first_setpoint = None
                # self.first_setpoint = (5.0, 5.0, 3.0)
                self.second_setpoint = None
                self.landed = None

                # Define waypoints as a list of tuples (x, y, z)
                self.waypoints = [
                (0.0, 0.0, self.takeoff_height),  # Takeoff position
                (10.0, 0.0, self.takeoff_height),  # First waypoint
                (10.0, 10.0, self.takeoff_height),  # Second waypoint
                (0.0, 10.0, self.takeoff_height),  # Third waypoint
                (0.0, 0.0, self.takeoff_height)  # Back to initial position
                ]
                self.current_waypoint_index = 0  # Start with the first waypoint

        def vehicle_local_position_callback(self, vehicle_local_position):
                """Callback function for vehicle_local_position topic subscriber"""
                self.vehicle_local_position = vehicle_local_position
        
        def vehicle_status_callback(self, vehicle_status):
                """Callback function for vehicle_status topic subscriber."""
                self.vehicle_status = vehicle_status

        def vehicle_odometry_callback(self, vehicle_odometry):
                """Callback function for vehicle_status topic subscriber."""
                self.vehicle_odometry = vehicle_odometry

        def vehicle_command_ack_callback(self, vehicle_command_ack):
                """Callback function for vehicle_status topic subscriber."""
                self.vehicle_command_ack= vehicle_command_ack

        def arm(self):
                """Send an arm command to the vehicle."""
                self.publish_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self.get_logger().info('Arm command sent')
        
        def disarm(self):
                """Send a disarm command to the vehicle."""
                self.publish_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
                self.get_logger().info('Disarm command sent')

        def engage_offboard_mode(self):
                """Switch to offboard mode."""
                self.publish_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self.get_logger().info("Switching to offboard mode")

        def land(self):
                """Switch to land mode."""
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.get_logger().info("Switching to land mode")

        def publish_offboard_control_heartbeat_signal(self):
                """Publish the offboard control mode."""
                msg = OffboardControlMode()
                msg.position = True 
                msg.velocity = True
                msg.acceleration = False
                msg.attitude = False
                msg.body_rate = False
                msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.offboard_control_mode_publisher.publish(msg)

        def publish_position_setpoint(self, x: float, y: float, z: float):
                """Publish the trajectory setpoint."""
                msg = TrajectorySetpoint()
                msg.position = [x, y, z]
                msg.yaw = 1.57079  # (90 degree)
                msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.trajectory_setpoint_publisher.publish(msg)
                self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

        def publish_vehicle_command(self, command, **params) -> None:
                """Publish a vehicle command."""
                msg = VehicleCommand()
                msg.command = command
                msg.param1 = params.get("param1", 0.0)
                msg.param2 = params.get("param2", 0.0)
                msg.param3 = params.get("param3", 0.0)
                msg.param4 = params.get("param4", 0.0)
                msg.param5 = params.get("param5", 0.0)
                msg.param6 = params.get("param6", 0.0)
                msg.param7 = params.get("param7", 0.0)
                msg.target_system = 1
                msg.target_component = 1
                msg.source_system = 1
                msg.source_component = 1
                msg.from_external = True
                msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.vehicle_command_publisher.publish(msg)

        def publish_velocity_setpoint(self, vx, vy, vz):
                """Publish a velocity setpoint."""
                msg = TrajectorySetpoint()
                msg.position = [np.nan, np.nan, np.nan]
                msg.velocity = [vx, vy, vz]
                msg.acceleration = [np.nan, np.nan, np.nan]
                msg.jerk = [np.nan, np.nan, np.nan]
                msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.trajectory_setpoint_publisher.publish(msg)
                self.get_logger().info(f"Publishing velocity setpoint: [{vx}, {vy}, {vz}]")

        def publish_vehicle_command_vtol_transition(self, transition_type) -> None:
                VTOL_TRANSITION_COMMAND = VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION

                self.publish_vehicle_command(command = VTOL_TRANSITION_COMMAND,
                                             param1 = float(transition_type),
                                             target_system = 1, 
                                             target_component = 1,
                                             source_system = 1,
                                             source_component = 1,
                                             from_external = True)
                
                self.get_logger().info(f"VTOL transition command sent. Transition Type: {transition_type}")

        def timer_callback(self) -> None:
                """Callback function for the timer."""

                self.publish_offboard_control_heartbeat_signal()

                if self.offboard_setpoint_counter == 10:
                    self.engage_offboard_mode()
                    self.arm()

                if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.taken_off is None:
                    print("publishing takeoff")
                    self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
                    self.publish_velocity_setpoint(0.0, 0.0, -2.50)
                    if self.vehicle_local_position.z < self.takeoff_height + 0.5:
                        self.taken_off = True
                        print("taken off")

                # # Move to the next waypoint if current waypoint is reached
                # if self.taken_off and self.current_waypoint_index < len(self.waypoints):
                #         target = self.waypoints[self.current_waypoint_index]
                #         self.publish_position_setpoint(*target)
                #         self.get_logger().info(f"Moving to waypoint {self.current_waypoint_index + 1}: {target}")
                        
                #         # Check if the drone is close to the target waypoint
                #         if (abs(self.vehicle_local_position.x - target[0]) < 0.2 and
                #                 abs(self.vehicle_local_position.y - target[1]) < 0.2 and
                #                 abs(self.vehicle_local_position.z - target[2]) < 0.2):
                #                 self.current_waypoint_index += 1

                if self.vehicle_local_position.z < self.takeoff_height + 0.2 and self.first_setpoint is None:
                    self.publish_position_setpoint(400.0, 0.0, self.takeoff_height)
                    self.publish_velocity_setpoint(10.0, 0.0, 0.0)
                    if self.vehicle_local_position.x >= 39.5: 
                           self.first_setpoint = True 
                           print("first setpoint")

                if self.offboard_setpoint_counter < 11:
                       self.offboard_setpoint_counter += 1

                # # Land the vehicle after reaching the last waypoint
                # if self.current_waypoint_index == len(self.waypoints):
                #         self.land()
                #         self.get_logger().info("Landing initiated.")

def main(args=None) -> None:
        print('Starting offboard control node...')
        rclpy.init(args=args)
        offboard_control = OffboardControl()
        rclpy.spin(offboard_control)
        offboard_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
        try:
            main()
        except Exception as e:
            print(e)
