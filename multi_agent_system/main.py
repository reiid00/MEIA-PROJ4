from spade import quit_spade

from agents.app_agent import AppAgent
from agents.traffic_control_station_agent import TrafficControlStationAgent
from agents.charging_control_station_agent import ChargingControlStationAgent
from agents.drone_agent import DroneAgent
from agents.dispatcher_agent import DispatcherAgent

from config import AGENT_NAMES, NUM_DRONES

if __name__ == "__main__":
    # Create and start the agents
    app_agent = AppAgent(f'{AGENT_NAMES["APP"]}@localhost', "password")
    traffic_control_station_agent = TrafficControlStationAgent(f'{AGENT_NAMES["TRAFFIC_CONTROL_STATION"]}@localhost', "password")
    charging_control_station_agent = ChargingControlStationAgent(f'{AGENT_NAMES["CHARGING_CONTROL_STATION"]}@localhost', "password")
    drone_agents = [
        DroneAgent(f'{AGENT_NAMES["DRONE"]}{i}@localhost', "password") for i in range(1, NUM_DRONES + 1)
    ]
    dispatcher_agent = DispatcherAgent(f'{AGENT_NAMES["DISPATCHER"]}@localhost', "password")

    # Start the agents
    app_agent.start()
    traffic_control_station_agent.start()
    charging_control_station_agent.start()
    for drone_agent in drone_agents:
        drone_agent.start()
    dispatcher_agent.start()

    try:
        while traffic_control_station_agent.is_alive():
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
