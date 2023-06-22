
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
from .drone_agent import DroneAgent
from .config import AGENT_NAMES, NUM_DRONES, HEIGHT_RANGE
from .utils import create_lp_cs_locations, allocate_shortest_station
import spade

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
    drone_agents = []
    for i in range(1, NUM_DRONES + 1):
        drone_agents.append(DroneAgent(f'{AGENT_NAMES["DRONE"]}{i}@192.168.1.91', "admin", i))
    return drone_agents

def init_drone_arrays(drone_agents, station_array):
    drone_controls = []
    drone_listeners = []
    for i in range(1, NUM_DRONES + 1):
        drone_listeners.append(DroneListener(i))
        rclpy.spin_once(drone_listeners[i-1])
        drone_x, drone_y = allocate_drone_initial(drone_listeners[i-1].current_position[0], drone_listeners[i-1].current_position[1], i, station_array)
        drone_controls.append(DroneControl(i,[drone_x,drone_y,-max((HEIGHT_RANGE[0] + ((i - 1) * 5)), HEIGHT_RANGE[1])], drone_listeners[i-1].current_position,DroneTargetType.LAND_PAD.value, drone_agents[i-1]))
    return drone_controls, drone_listeners, drone_agents

async def run_agents(drone_agents):
    for agent in drone_agents:
      await agent.start(auto_register=True)

    await spade.wait_until_finished(drone_agents[0])

    stop_agents(drone_agents)

async def stop_agents(drone_agents):
    for agent in drone_agents:
      await agent.stop()

def run_spade(drone_agents):
    spade.run(run_agents(drone_agents))

def init_lp_cs_listeners():
    lp_listeners = []
    cs_listeners = []
    land_pads, charging_stations = create_lp_cs_locations()
    for i in range(1,5):
        for j in range(1,5):
            x_lp, y_lp = land_pads[i][j]
            x_cs, y_cs = charging_stations[i][j]
            lp_listeners.append(LandPadListener(i, j, x_lp, y_lp))
            cs_listeners.append(ChargingSpotListener(i, j, x_cs, y_cs))
    return lp_listeners, cs_listeners

def main(args=None):
    rclpy.init(args=args)
    drone_agents = init_spade_agents()
    spade_running = threading.Thread(target= run_spade, args=(drone_agents,))
    spade_running.start()  
    lp_listeners, cs_listeners = init_lp_cs_listeners()
    drone_controls, drone_listeners, drone_agents = init_drone_arrays(drone_agents, lp_listeners)
    drones_running = threading.Thread(target= run_drones, args=(drone_controls,drone_listeners,lp_listeners,cs_listeners,))
    drones_running.start()  


if __name__ == '__main__':
    spade.run(main())