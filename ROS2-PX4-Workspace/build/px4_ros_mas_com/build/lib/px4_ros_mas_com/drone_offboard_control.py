import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleLocalPosition


class DroneControl(Node):

    def __init__(self, drone_num=1):
        super().__init__('OffboardControl')
        topic_drone = f"/px4_{drone_num}"
        qos_profile = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                        f"{topic_drone}/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    f"{topic_drone}/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, f"{topic_drone}/fmu/in/vehicle_command", 10)


       # self.vehicle_local_pos_listener_ = self.create_subscription(VehicleLocalPosition, f"{topic_drone}/fmu/out/vehicle_local_position", self.listener_callback, qos_profile)

        self.current_position = [0.0, 0.0, 0.0]
        self.offboard_setpoint_counter_ = 0

        self.target_system = drone_num+1

        timer_period = 0.1  # 100 milliseconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if (self.offboard_setpoint_counter_ == 10):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            # Arm the vehicle
            self.arm()
        # stop the counter after reaching 11
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint(0.0, 0.0, -5.0)  # initial height set to 5 meters
        if (self.offboard_setpoint_counter_ < 11):
            self.offboard_setpoint_counter_ += 1
    
    def fly_to_position(self, lat=0.0, long=0.0, height=0.0):
        # Send the drone to the desired height first.
        self.get_logger().info("Fly To Pos")
        desired_height = height
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint(0.0, 0.0, desired_height)
        
        # Wait until the drone's current height is close to the desired height.
        while abs(self.current_position[2] - desired_height) > 0.1:  # Change 0.1 to desired tolerance.
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, desired_height)
            self.get_logger().info(f'{self.current_position[2]}')
            rclpy.spin_once(self)
        self.publish_trajectory_setpoint(lat, long, height)
    
    #def listener_callback(self, msg):
    #    self.current_position = [msg.x, msg.y, msg.z]
    #    print(self.current_position)


    # Arm the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    # Disarm the vehicle
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")

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

    def publish_trajectory_setpoint(self, lat=0.0, long=0.0, height=0.0, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [lat, long, -height] 
        msg.yaw = yaw  # [-PI:PI]
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



def main(args=None):
    rclpy.init(args=args)
    print("Starting offboard control node...\n")
    offboard_control = DroneControl()
    #rclpy.spin_once(offboard_control)
    #offboard_control.fly_to_position(20.0,10.0,-10.0)

    rclpy.spin(offboard_control)
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()