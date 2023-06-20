
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

from .droneControl import DroneControl
from .droneListener import DroneListener
from .landPadListener import LandPadListener
from .chargingSpotListener import ChargingSpotListener

from std_msgs.msg import String 

import time
import threading


def run_drones(drones, drones_listeners, land_pad_listeners, charging_station_listeners):
    while rclpy.ok():
        for drone in drones:
            drone_listener = next((drone_listener for drone_listener in drones_listeners if drone_listener.drone_num == drone.drone_num), None)
            drone.update_current_position(drone_listener.current_position)
            rclpy.spin_once(drone)
            if drone.disarmmed : drones.remove(drone)
        for drone in drones_listeners:
            rclpy.spin_once(drone)
        for land_pad in land_pad_listeners:
            rclpy.spin_once(land_pad)
        for charging_station in charging_station_listeners:
            rclpy.spin_once(charging_station)


def get_available_drones(drones_listeners, drones_running):
    available_drones = []
    for drone_listener in drones_listeners:
        drone = next((drone_running for drone_running in drones_running if drone_running.drone_num == drone_listener.drone_num), None)
        if drone == None : available_drones.append(drone_listener)
    return available_drones


def listen_drones_once(drones_listeners):
    for drone in drones_listeners:
        rclpy.spin_once(drone)

def main(args=None):
    rclpy.init(args=args)
    drones_pos_listener = []
    drone_pos_listener = DroneListener(2)
    drone_pos_listener2 = DroneListener(1)
    drones_pos_listener.append(drone_pos_listener)
    drones_pos_listener.append(drone_pos_listener2)

    listen_drones_once(drones_pos_listener)
    drone_controls = []
    offboard_control = DroneControl(2,[19.5,19.5,-10.0], drone_pos_listener2.current_position)
    offboard_control2 = DroneControl(1,[19.5,0.5,-10.0], drone_pos_listener.current_position)
    drone_controls.append(offboard_control)
    drone_controls.append(offboard_control2)

    land_pad_listeners = []
    charging_station_listeners = []
    for i in range(1,5):
        for j in range(1,5):
            land_pad_listeners.append(LandPadListener(i,j))
            charging_station_listeners.append(ChargingSpotListener(i,j))
    drones_running = threading.Thread(target= run_drones, args=(drone_controls,drones_pos_listener,land_pad_listeners,charging_station_listeners,))
    drones_running.start()  



    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #offboard_control.destroy_node()
    #offboard_control2.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()