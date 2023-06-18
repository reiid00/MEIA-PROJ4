import time

from app_agent import AppAgent
from charging_control_station_agent import ChargingControlStationAgent
from common.config import AGENT_NAMES, NUM_DRONES
from dispatcher_agent import DispatcherAgent
from drone_agent import DroneAgent
from spade import quit_spade
from traffic_control_station_agent import TrafficControlStationAgent

if __name__ == "__main__":
    # Create and start the agents
    app_agent = AppAgent(f'{AGENT_NAMES["APP"]}@localhost', "password")
    traffic_control_station_agent = TrafficControlStationAgent(f'{AGENT_NAMES["TRAFFIC_CONTROL_STATION"]}@localhost', "password")
    charging_control_station_agent = ChargingControlStationAgent(f'{AGENT_NAMES["CHARGING_CONTROL_STATION"]}@localhost', "password")
    drone_agents = [
        DroneAgent(f'{AGENT_NAMES["DRONE"]}{i}@localhost', "password", i) for i in range(1, NUM_DRONES + 1)
    ]
    dispatcher_agent = DispatcherAgent(f'{AGENT_NAMES["DISPATCHER"]}@localhost', "password")

    # Start the agents and wait for them to be prepared (result does this check)
    future_app_agent = app_agent.start()
    future_app_agent.result()
    future_traffic_control_station_agent = traffic_control_station_agent.start()
    future_traffic_control_station_agent.result()
    future_charging_control_station_agent = charging_control_station_agent.start()
    future_charging_control_station_agent.result()
    for drone_agent in drone_agents:
        future_drone_agent = drone_agent.start()
        future_drone_agent.result()
    future_dispatcher_agent = dispatcher_agent.start()
    future_dispatcher_agent.result()

    try:
        while traffic_control_station_agent.is_alive():
            time.sleep(1)
            pass
    except KeyboardInterrupt:
        print("Stopping...")
        
        # Stop the agents and the Spade platform
        app_agent.stop()
        traffic_control_station_agent.stop()
        charging_control_station_agent.stop()
        for drone_agent in drone_agents:
            drone_agent.stop()
        dispatcher_agent.stop()
        quit_spade()
