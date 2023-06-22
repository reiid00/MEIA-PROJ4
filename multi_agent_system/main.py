import spade
from app_agent import AppAgent
from charging_control_station_agent import ChargingControlStationAgent
from common.config import AGENT_NAMES, XMPP_SERVER_URL
from dispatcher_agent import DispatcherAgent
from traffic_control_station_agent import TrafficControlStationAgent


async def main():
    # Create and start the agents
    app_agent = AppAgent(f'{AGENT_NAMES["APP"]}@{XMPP_SERVER_URL}', "admin")
    traffic_control_station_agent = TrafficControlStationAgent(f'{AGENT_NAMES["TRAFFIC_CONTROL_STATION"]}@{XMPP_SERVER_URL}', "admin")
    charging_control_station_agent = ChargingControlStationAgent(f'{AGENT_NAMES["CHARGING_CONTROL_STATION"]}@{XMPP_SERVER_URL}', "admin")
    dispatcher_agent = DispatcherAgent(f'{AGENT_NAMES["DISPATCHER"]}@{XMPP_SERVER_URL}', "admin")

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