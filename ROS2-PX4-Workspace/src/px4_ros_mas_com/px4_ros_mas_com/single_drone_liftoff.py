
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

from .droneControl import DroneControl, DroneTargetType
from .droneListener import DroneListener
from .landPadListener import LandPadListener
from .chargingSpotListener import ChargingSpotListener

from std_msgs.msg import String 

from .utils import create_lp_cs_locations, allocate_shortest_station

import time
import threading



def run_drones(drones, drones_listeners, land_pad_listeners, charging_station_listeners):
    while rclpy.ok():
        for drone in drones:
            drone_listener = next((drone_listener for drone_listener in drones_listeners if drone_listener.drone_num == drone.drone_num), None)
            drone.update_current_position(drone_listener.current_position)
            rclpy.spin_once(drone)
            if drone.disarmmed : drones.remove(drone)
            if drone.needs_allocation: allocate_drone(drone, land_pad_listeners) 
        for drone in drones_listeners:
            rclpy.spin_once(drone)
        for land_pad in land_pad_listeners:
            rclpy.spin_once(land_pad)
        for charging_station in charging_station_listeners:
            rclpy.spin_once(charging_station)

def allocate_drone(drone, station_array):
    drone_pos = drone.return_current_pos()
    station_to_allocate = allocate_shortest_station(drone_pos[0],drone_pos[1], station_array)
    print(f"Drone {drone.drone_num} allocated to {station_to_allocate.get_station()} ")
    drone.update_location(station_to_allocate.get_type, station_to_allocate.x, station_to_allocate.y, -20.0)
    station_to_allocate.assign_drone()

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
    offboard_control = DroneControl(2,[30.0,30.0,-10.0], drone_pos_listener.current_position,DroneTargetType.CUSTOMER.value)
    offboard_control2 = DroneControl(1,[-55.0,1.0,-10.0], drone_pos_listener2.current_position,DroneTargetType.CUSTOMER.value)
    drone_controls.append(offboard_control)
    drone_controls.append(offboard_control2)

    lp_listeners = []

    cs_listeners = []
    
    land_pads, charging_stations = create_lp_cs_locations()
    for i in range(1,5):
        for j in range(1,5):
            x_lp, y_lp = land_pads[i][j]
            x_cs, y_cs = charging_stations[i][j]
            lp_listeners.append(LandPadListener(i, j, x_lp, y_lp))
            cs_listeners.append(ChargingSpotListener(i, j, x_cs, y_cs))
    drones_running = threading.Thread(target= run_drones, args=(drone_controls,drones_pos_listener,lp_listeners,cs_listeners,))
    drones_running.start()  


if __name__ == '__main__':
    main()