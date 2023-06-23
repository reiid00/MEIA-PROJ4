
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
from multi_agent_system.drone_agent import DroneAgent
from multi_agent_system.common.config import AGENT_NAMES, NUM_DRONES, HEIGHT_RANGE, XMPP_SERVER_URL
from .utils import create_lp_cs_locations, allocate_shortest_station
from multi_agent_system.ros2_node_agent import ROS2NodeAgent
from multi_agent_system.charging_control_station_agent import ChargingControlStationAgent
from multi_agent_system.dispatcher_agent import DispatcherAgent
from multi_agent_system.traffic_control_station_agent import TrafficControlStationAgent
from multi_agent_system.app_agent import AppAgent

import spade

import threading

def handle_target_reach_confirmation(drone_id, target):
    DRONE_CONTROLS[drone_id-1].update_location(
        targetType= target.type, 
        x = float(target.location.latitude), 
        y = float(target.location.longitude), 
        z = - float(target.route_height)
    )

def handle_charging_completion_confirmation(drone_id):
    allocate_drone(DRONE_CONTROLS[drone_id-1], LP_LISTENERS)

def run_drones():
    while rclpy.ok():
        for drone in DRONE_CONTROLS:
            drone_listener = next((drone_listener for drone_listener in DRONE_LISTENERS if drone_listener.drone_num == drone.drone_num), None)
            drone.update_current_position(drone_listener.current_position)
            rclpy.spin_once(drone)
            if drone.disarmmed : DRONE_CONTROLS.remove(drone)
            if drone.needs_allocation: allocate_drone(drone, LP_LISTENERS) 
        for drone in DRONE_LISTENERS:
            rclpy.spin_once(drone)
        for land_pad in LP_LISTENERS:
            rclpy.spin_once(land_pad)
        for charging_station in CS_LISTENERS:
            rclpy.spin_once(charging_station)

def allocate_drone(drone, station_array):
    drone_pos = drone.return_current_pos()
    station_to_allocate = allocate_shortest_station(drone_pos[0],drone_pos[1], station_array)
    print(f"[INFO] [2734655362.1823940596] [ControlAgent] Drone {drone.drone_num} allocated to {station_to_allocate.get_station()} ")
    drone.update_location(station_to_allocate.get_type, station_to_allocate.x, station_to_allocate.y, -20.0)
    station_to_allocate.assign_drone()


def allocate_drone_initial(drone_pos_x, drone_pos_y, drone_num, station_array):
    station_to_allocate = allocate_shortest_station(drone_pos_x,drone_pos_y, station_array)
    print(f"[INFO] [2734655362.1823940596] [ControlAgent] Drone {drone_num} allocated to {station_to_allocate.get_station()} ")
    station_to_allocate.assign_drone()
    return station_to_allocate.x, station_to_allocate.y


def get_available_drones(drones_listeners, drones_running):
    available_drones = []
    for drone_listener in drones_listeners:
        drone = next((drone_running for drone_running in drones_running if drone_running.drone_num == drone_listener.drone_num), None)
        if drone == None : available_drones.append(drone_listener)
    return available_drones

def init_spade_agents():
    for i in range(1, NUM_DRONES + 1):
        DRONE_AGENTS.append(DroneAgent(f'{AGENT_NAMES["DRONE"]}{i}@{XMPP_SERVER_URL}', "admin", i))

def drone_reach_location(drone_id):
    ROS2_NODE_AGENT.target_reached[drone_id] = True

def init_drone_arrays():
    for i in range(1, NUM_DRONES + 1):
        DRONE_LISTENERS.append(DroneListener(i))
        rclpy.spin_once(DRONE_LISTENERS[i-1])
        drone_x, drone_y = allocate_drone_initial(DRONE_LISTENERS[i-1].current_position[0], DRONE_LISTENERS[i-1].current_position[1], i, LP_LISTENERS)
        DRONE_CONTROLS.append(DroneControl(i,[drone_x,drone_y,-max((HEIGHT_RANGE[0] + ((i - 1) * 5)), HEIGHT_RANGE[1]) / 10.0], DRONE_LISTENERS[i-1].current_position,DroneTargetType.LAND_PAD.value, DRONE_AGENTS[i-1], drone_reach_location))

async def run_agents():
    for agent in DRONE_AGENTS:
      await agent.start(auto_register=True)
    
    await ROS2_NODE_AGENT.start(auto_register=True)
    await APP_AGENT.start(auto_register=True)
    await TRAFFIC_CONTROL_STATION_AGENT.start(auto_register=True)
    await CHARGING_CONTROL_STATION_AGENT.start(auto_register=True)
    await DISPATCHER_AGENT.start(auto_register=True)

    await spade.wait_until_finished(TRAFFIC_CONTROL_STATION_AGENT)

    stop_agents()

async def stop_agents():
    for agent in DRONE_AGENTS:
      await agent.stop()
    await ROS2_NODE_AGENT.stop()
    await APP_AGENT.stop()
    await TRAFFIC_CONTROL_STATION_AGENT.stop()
    await CHARGING_CONTROL_STATION_AGENT.stop()
    await DISPATCHER_AGENT.stop()

def run_spade():
    spade.run(run_agents())

def init_lp_CS_LISTENERS():
    land_pads, charging_stations = create_lp_cs_locations()
    for i in range(1,5):
        for j in range(1,5):
            x_lp, y_lp = land_pads[i][j]
            x_cs, y_cs = charging_stations[i][j]
            LP_LISTENERS.append(LandPadListener(i, j, x_lp, y_lp))
            CS_LISTENERS.append(ChargingSpotListener(i, j, x_cs, y_cs))

DRONE_CONTROLS = []
DRONE_LISTENERS = []
DRONE_AGENTS = []
LP_LISTENERS = []
CS_LISTENERS = []
ROS2_NODE_AGENT = ROS2NodeAgent(f'{AGENT_NAMES["ROS2_NODE"]}@{XMPP_SERVER_URL}', "admin", handle_target_reach_confirmation, handle_charging_completion_confirmation)
APP_AGENT = AppAgent(f'{AGENT_NAMES["APP"]}@{XMPP_SERVER_URL}', "admin")
TRAFFIC_CONTROL_STATION_AGENT = TrafficControlStationAgent(f'{AGENT_NAMES["TRAFFIC_CONTROL_STATION"]}@{XMPP_SERVER_URL}', "admin")
CHARGING_CONTROL_STATION_AGENT = ChargingControlStationAgent(f'{AGENT_NAMES["CHARGING_CONTROL_STATION"]}@{XMPP_SERVER_URL}', "admin")
DISPATCHER_AGENT = DispatcherAgent(f'{AGENT_NAMES["DISPATCHER"]}@{XMPP_SERVER_URL}', "admin")

def main(args=None):
    rclpy.init(args=args)
    init_spade_agents()
    spade_running = threading.Thread(target= run_spade, args=())
    spade_running.start()  
    init_lp_CS_LISTENERS()
    init_drone_arrays()
    drones_running = threading.Thread(target= run_drones, args=())
    drones_running.start()  


if __name__ == '__main__':
    spade.run(main())