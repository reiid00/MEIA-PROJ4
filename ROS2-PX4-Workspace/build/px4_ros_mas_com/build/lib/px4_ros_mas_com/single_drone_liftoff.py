import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import SensorGps

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

    def __init__(self, drone_num=1):
        super().__init__("vehicle_global_position_listener")
        self.drone_num = drone_num
        self.topic_drone = f"/px4_{drone_num}"
        
        qos_profile = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)

        self.current_position = [0.0, 0.0, 0.0]

        self.vehicle_local_pos_listener_ = self.create_subscription(SensorGps, f"{self.topic_drone}/fmu/out/vehicle_gps_position", self.listener_callback, qos_profile)

    def listener_callback(self, msg):
        print("\n" * 10)
        print("RECEIVED VEHICLE GPS POSITION DATA")
        print("==================================")
        print("ts: ", msg.timestamp)
        print("lat: ", msg.lat)
        print("lon: ", msg.lon)
        print("alt: ", msg.alt)
        print("alt_ellipsoid: ", msg.alt_ellipsoid)
        print("s_variance_m_s: ", msg.s_variance_m_s)
        print("c_variance_rad: ", msg.c_variance_rad)
        print("fix_type: ", msg.fix_type)
        print("eph: ", msg.eph)
        print("epv: ", msg.epv)
        print("hdop: ", msg.hdop)
        print("vdop: ", msg.vdop)
        print("noise_per_ms: ", msg.noise_per_ms)
        print("vel_m_s: ", msg.vel_m_s)
        print("vel_n_m_s: ", msg.vel_n_m_s)
        print("vel_e_m_s: ", msg.vel_e_m_s)
        print("vel_d_m_s: ", msg.vel_d_m_s)
        print("cog_rad: ", msg.cog_rad)
        print("vel_ned_valid: ", msg.vel_ned_valid)
        print("timestamp_time_relative: ", msg.timestamp_time_relative)
        print("time_utc_usec: ", msg.time_utc_usec)
        print("satellites_used: ", msg.satellites_used)
        print("heading: ", msg.heading)
        print("heading_offset: ", msg.heading_offset)

class VehicleGpsPositionListener(Node):
    def __init__(self):
        super().__init__('vehicle_global_position_listener')
        qos_profile = QoSProfile(depth=1)

        self.subscription = self.create_subscription(SensorGps, '/px4_2/fmu/out/vehicle_gps_position', lambda msg: print(msg), qos_profile)

    

def run_drones(drones, drones_listeners):
    while rclpy.ok():
        for drone in drones_listeners:
            rclpy.spin_once(drone)
            print(f"drone: {drone.drone_num}, coordinates: {drone.current_position}")


def main(args=None):
    #rclpy.init(args=args)
    #print("Starting offboard control node...\n")
    
    print("Starting vehicle_global_position listener node...")
    rclpy.init(args=args)
    vehicle_gps_position_listener = VehicleGpsPositionListener()
    rclpy.spin(vehicle_gps_position_listener)
    vehicle_gps_position_listener.destroy_node()
    rclpy.shutdown()
    #drone_controls = []
    #offboard_control = DroneControl(2,0.0,0.0,-10.0)
    #offboard_control2 = DroneControl(1,0.0,0.0,-20.0)
    #drone_controls.append(offboard_control)
    #drone_controls.append(offboard_control2)
    #drones_pos_listener = []
    #drone_pos_listener = DronePosListener(2)
    #drone_pos_listener2 = DronePosListener(1)
    #drones_pos_listener.append(drone_pos_listener)
    #drones_pos_listener.append(drone_pos_listener2)
    #drones_running = threading.Thread(target= run_drones, args=(drone_controls,drones_pos_listener,))
    #drones_running.start()  



    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #offboard_control.destroy_node()
    #offboard_control2.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()