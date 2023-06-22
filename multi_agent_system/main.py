import spade
from app_agent import AppAgent
from charging_control_station_agent import ChargingControlStationAgent
from common.config import AGENT_NAMES, XMPP_SERVER_URL
from dispatcher_agent import DispatcherAgent
from traffic_control_station_agent import TrafficControlStationAgent


async def main():
    # Create and start the agents
<<<<<<< HEAD:multi_agent_system/main.py
    app_agent = AppAgent(f'{AGENT_NAMES["APP"]}@{XMPP_SERVER_URL}', "admin")
    traffic_control_station_agent = TrafficControlStationAgent(f'{AGENT_NAMES["TRAFFIC_CONTROL_STATION"]}@{XMPP_SERVER_URL}', "admin")
    charging_control_station_agent = ChargingControlStationAgent(f'{AGENT_NAMES["CHARGING_CONTROL_STATION"]}@{XMPP_SERVER_URL}', "admin")
    dispatcher_agent = DispatcherAgent(f'{AGENT_NAMES["DISPATCHER"]}@{XMPP_SERVER_URL}', "admin")
=======
    app_agent = AppAgent(f'{AGENT_NAMES["APP"]}@192.168.1.91', "admin")
    traffic_control_station_agent = TrafficControlStationAgent(f'{AGENT_NAMES["TRAFFIC_CONTROL_STATION"]}@192.168.1.91', "admin")
    charging_control_station_agent = ChargingControlStationAgent(f'{AGENT_NAMES["CHARGING_CONTROL_STATION"]}@192.168.1.91', "admin")
    # drone_agents = [
    #     DroneAgent(f'{AGENT_NAMES["DRONE"]}{i}@192.168.1.91', "admin", i) for i in range(1, NUM_DRONES + 1)
    # ]
    dispatcher_agent = DispatcherAgent(f'{AGENT_NAMES["DISPATCHER"]}@192.168.1.91', "admin")
>>>>>>> 4ccfd5987fc7055259ae78f6c1a2d4b06bf1ab17:multi_agent_system/_main.py

   # Start the agents and wait for them to be prepared
    await app_agent.start(auto_register=True)
    await traffic_control_station_agent.start(auto_register=True)
    await charging_control_station_agent.start(auto_register=True)
    await dispatcher_agent.start(auto_register=True)

    await spade.wait_until_finished(traffic_control_station_agent)
        
    # Stop the agents and the Spade platform
    await app_agent.stop()
    await traffic_control_station_agent.stop()
    await charging_control_station_agent.stop()
    await dispatcher_agent.stop()

if __name__ == "__main__":
    spade.run(main())