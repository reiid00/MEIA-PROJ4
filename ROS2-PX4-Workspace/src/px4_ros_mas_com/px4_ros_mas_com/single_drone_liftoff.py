import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAttitude

from std_msgs.msg import String

import threading


class DroneControl(Node):

    def __init__(self, drone_num=1, x=0.0, y=0.0, z=0.0):
        super().__init__('OffboardControl')
        topic_drone = f"/px4_{drone_num}"
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                        f"{topic_drone}/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    f"{topic_drone}/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, f"{topic_drone}/fmu/in/vehicle_command", 10)

        self.offboard_setpoint_counter_ = 0

        self.target_system = drone_num+1

        self.x = x
        self.y = y
        self.z = z

        self.current_position = [0.0, 0.0, 0.0]
        self.goal_position = [0.0, 0.0, 0.0]

        timer_period = 0.1  # 100 milliseconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if (self.offboard_setpoint_counter_ == 10):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            # Arm the vehicle
            self.arm()

        # Offboard_control_mode needs to be paired with trajectory_setpoint
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint(self.x,self.y,self.z)

        # stop the counter after reaching 11
        if (self.offboard_setpoint_counter_ < 11):
            self.offboard_setpoint_counter_ += 1

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

    def publish_trajectory_setpoint(self, x = 0.0, y = 0.0, z = 0.0):
        msg = TrajectorySetpoint()
        msg.position = [x,y,z] 
        msg.yaw = -3.14  # [-PI:PI]
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
    



class DronePosListener(Node):

    def __init__(self, drone_num = 1):
        super().__init__('minimal_subscriber')
        self.drone_num = drone_num
        topic_drone = f"/px4_{drone_num}"

        qos_profile = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE )

        self.current_position = [0.0, 0.0, 0.0]

        self.vehicle_local_pos_listener_ = self.create_subscription(VehicleLocalPosition, f"{topic_drone}/fmu/out/vehicle_local_position", self.listener_callback, qos_profile)


    def listener_callback(self, msg):
        self.current_position = [msg.x, msg.y, msg.z]
        self.get_logger().info(f"Current pos: {self.current_position} for Drone: {self.drone_num}")
    

def run_drones(drones, drones_listeners):
    while rclpy.ok():
        for drone in drones:
            rclpy.spin_once(drone)
        for drone in drones_listeners:
            rclpy.spin_once(drone)


def main(args=None):
    rclpy.init(args=args)

    drone_controls = []
    offboard_control = DroneControl(2,0.0,0.0,-10.0)
    offboard_control2 = DroneControl(1,0.0,0.0,-20.0)
    drone_controls.append(offboard_control)
    drone_controls.append(offboard_control2)
    drones_pos_listener = []
    offboard_control = DroneControl(2,0.0,0.0,-20.0)
    offboard_control2 = DroneControl(1,0.0,0.0,-10.0)
    drone_pos_listener = DronePosListener(2)
    drone_pos_listener2 = DronePosListener(1)
    drone_controls.append(offboard_control)
    drone_controls.append(offboard_control2)

    drones_pos_listener.append(drone_pos_listener)
    drones_pos_listener.append(drone_pos_listener2)
    drones_running = threading.Thread(target= run_drones, args=(drone_controls,drones_pos_listener,))
    drones_running.start()  



    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #offboard_control.destroy_node()
    #offboard_control2.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()