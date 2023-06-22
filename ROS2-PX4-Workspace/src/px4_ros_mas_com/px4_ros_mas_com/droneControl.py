import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from enum import Enum
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from rclpy.duration import Duration
import json

class DroneTargetType(Enum):
    DISPATCHER = 1
    CUSTOMER = 2
    CHARGING_STATION = 3
    LAND_PAD = 4


class DroneControl(Node):

    def __init__(self, drone_num=1, goal_position=[0.0,0.0,0.0], current_position=[0.0,0.0,0.0], target_type = "TESTING", drone_agent):
        super().__init__('DroneControl')
        self.drone_num = drone_num
        self.target_system = drone_num+1
        self.offboard_setpoint_counter_ = 0
        self.target_type = target_type

        topic_drone = f"/px4_{drone_num}"
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                        f"{topic_drone}/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    f"{topic_drone}/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, f"{topic_drone}/fmu/in/vehicle_command", 10)

        self.current_position = current_position
        self.goal_position = goal_position
        self.goal_position[1] = self.goal_position[1] - 3 * self.drone_num
        self.x = current_position[0]
        self.y = current_position[1]
        self.z = self.goal_position[2]
        self.final_height = -2.0 # Can Change

        self.reached_height = False
        self.reached_goal = False
        self.served_purpose = False
        self.disarmmed = False

        self.response_timer_ = None
        self.needs_allocation = False

        timer_period = 0.1  # 100 milliseconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        self.drone_agent = drone_agent

        
    def timer_callback(self):

        if (self.offboard_setpoint_counter_ == 10):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            # Arm the vehicle
            self.arm()

        # Initially go to the wanted Height, set goal_postion to destination
        if not self.reached_height and self.validate_height(self.goal_position[2], self.current_position[2]):
            self.reached_height = True
            self.y = self.goal_position[0]
            self.x = self.goal_position[1]
            self.get_logger().info(f'Drone {self.drone_num} reached preferable height of: {abs(self.goal_position[2])}, going to destiny!')
        
        # Travel to the destination, set Height to zero
        elif self.reached_height and not self.reached_goal and self.validate_goal_pos(self.goal_position[1], self.goal_position[0], self.current_position[0], self.current_position[1]):
            self.reached_goal = True
            self.z = self.final_height
            self.get_logger().info(f'Drone {self.drone_num} reached preferable destination, going to the ground!')


        # Disarm drone when reach floor
        if self.reached_goal and self.validate_height(self.final_height, self.current_position[2]):
            if self.target_type == DroneTargetType.DISPATCHER.value and not self.served_purpose:
                self.served_purpose = True
                self.send_request("pickup")
                self.start_response_timer()
                self.get_logger().info(f'Drone {self.drone_num} reached dispatcher height, waiting for package pickup!')
            if self.target_type == DroneTargetType.CUSTOMER.value and not self.served_purpose:
                self.served_purpose = True
                self.send_request("deliver")
                self.start_response_timer()
                self.get_logger().info(f'Drone {self.drone_num} reached customer height, waiting for package delivery!')
                #Start Timer
            elif not self.disarmmed and self.target_type != DroneTargetType.CUSTOMER.value and self.target_type != DroneTargetType.DISPATCHER.value:
                self.get_logger().info(f'Drone {self.drone_num} reached the ground, disarmming!')
                self.disarm()
                self.disarmmed=True

        # Offboard_control_mode needs to be paired with trajectory_setpoint
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint(self.x,self.y,self.z)

        # stop the counter after reaching 11
        if (self.offboard_setpoint_counter_ < 11):
            self.offboard_setpoint_counter_ += 1

    def send_request(self, string):
        pass

    def start_response_timer(self):
        self.response_timer_ = self.create_timer(10, self.response_timer_callback)

    def stop_response_timer(self):
        self.response_timer_.cancel()
        self.response_timer_ = None

    def response_timer_callback(self):
        self.get_logger().info(f'Drone {self.drone_num} did not receive the expected response in {10} seconds, Requesting Land Pad!')
        self.needs_allocation = True
        self.stop_response_timer()

    def update_location(self, targetType, x, y, z):

        if self.response_timer_ is not None:
            self.stop_response_timer()

        # In case drone is disarmmed
        if self.disarmmed:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            self.arm()
            
        self.goal_position = [x , y - 3 * self.drone_num, z]
        self.z = z
        self.target_type = targetType

        self.reached_height = False
        self.reached_goal = False
        self.served_purpose = False
        self.needs_allocation = False

        
    

    def calculate_angle(self):
        pass

    def return_current_pos(self):
        return [self.current_position[1],self.current_position[0] + 3 * self.drone_num, self.current_position[2]]

    def update_current_position(self, current_position):
        self.current_position = current_position
        self.drone_agent.location = json.dumps({'latitude': self.current_position[1], 'longitude': self.current_position[0] + 3 * self.drone_num})

    def validate_height(self, z, current_z):
        return abs((abs(z) - abs(current_z))) < 2e-1
    
    def validate_goal_pos(self, x, y, current_x, current_y):
        return abs(x) - abs(current_x) < 1e-2 and (abs(y) - abs(current_y)) < 1e-2


    # Arm the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info(f"Arm command send for Drone {self.drone_num}")

    # Disarm the vehicle
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info(f"Disarm command send for Drone {self.drone_num}")

    '''
	Publish the offboard control mode.
	For this example, only position and altitude controls are active.
    '''

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)

    '''
	Publish a trajectory setpoint
	For this example, it sends a trajectory setpoint to make the
	vehicle hover at 5 meters with a yaw angle of 180 degrees.
    '''

    def publish_trajectory_setpoint(self, x = 0.0, y = 0.0, z = 0.0):
        msg = TrajectorySetpoint()
        msg.position = [x,y,z] 
        msg.yaw = 0.0  # [-PI:PI]
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.trajectory_setpoint_publisher_.publish(msg)

    '''
    Publish vehicle commands
        command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
        param1    Command parameter 1 as defined by MAVLink uint16 VEHICLE_CMD enum
        param2    Command parameter 2 as defined by MAVLink uint16 VEHICLE_CMD enum
    '''
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command  # command ID
        msg.target_system = self.target_system   # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)
    

