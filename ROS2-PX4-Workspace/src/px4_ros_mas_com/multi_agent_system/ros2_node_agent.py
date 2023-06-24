import datetime
import json

from common.base_agent import BaseAgent
from common.config import (AGENT_NAMES, XMPP_SERVER_URL, NUM_DRONES)
from spade.behaviour import CyclicBehaviour, PeriodicBehaviour
from spade.message import Message


class ROS2NodeAgent(BaseAgent):
    def __init__(self, jid: str, password: str, handle_new_drone_target_received, handle_charging_completion_confirmation):
        super().__init__(jid, password)
        self.target_reached = {}  # Dictionary to store drone target reached confirmation flags
        # Set up drone target reached confirmation dictionary
        for i in range(1, NUM_DRONES + 1):
            drone_id = str(i)
            self.target_reached[drone_id] = False
        self.handle_new_drone_target_received = handle_new_drone_target_received
        self.handle_charging_completion_confirmation = handle_charging_completion_confirmation

    class DroneTargetReachedReportingBehaviour(PeriodicBehaviour):
        async def run(self):
            target_reached_drones = [drone_id for drone_id, reached_current_target in self.agent.target_reached.items() if reached_current_target == True]
            for drone_id in target_reached_drones:
                # Reset Flag
                self.agent.target_reached[drone_id] = False
                # Send target reached confirmation to corresponding Drone agent
                await self.send_target_reached_confirmation(drone_id)

        async def send_target_reached_confirmation(self, drone_id):
            confirmation_msg = Message(to=f'{AGENT_NAMES["DRONE"]}{drone_id}@{XMPP_SERVER_URL}')
            confirmation_msg.set_metadata("performative", "confirm_target_reached")
            confirmation_msg.body = json.dumps(True)
            await self.send(confirmation_msg)
            self.agent.agent_say(f'Target reached confirmation obtained, notified drone {drone_id}.')

    class DroneHandlingBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=10)
            if msg:
                performative = msg.get_metadata("performative")
                if performative == "inform_target":
                    # Receive new target information from Drone Agent
                    target_info = json.loads(msg.body)
                    # Handle target logic using the received new target information
                    self.agent.handle_new_drone_target_received(target_info["drone_id"], target_info["target"])
                elif performative == "confirm_charging_completion":
                    # Receive charging completion confirmation from Drone Agent
                    confirmation_info = json.loads(msg.body)
                    # Handle target reached logic using the received confirmation information
                    self.agent.handle_charging_completion_confirmation(confirmation_info["drone_id"])

    async def setup(self):
        await super().setup()
        start_at = datetime.datetime.now() + datetime.timedelta(seconds=5)
        self.add_behaviour(self.DroneTargetReachedReportingBehaviour(period=1, start_at=start_at))
        self.add_behaviour(self.DroneHandlingBehaviour())