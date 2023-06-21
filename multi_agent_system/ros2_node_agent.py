import datetime
import json

from common.base_agent import BaseAgent
from common.config import (AGENT_NAMES, DroneTargetType)
from spade.behaviour import CyclicBehaviour, PeriodicBehaviour
from spade.message import Message


class ROS2NodeAgent(BaseAgent):
    def __init__(self, jid: str, password: str):
        super().__init__(jid, password)

    class DroneHandlingBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=10)
            if msg:
                performative = msg.get_metadata("performative")
                if performative == "inform_target":
                    # Receive new target information from Drone Agent
                    target_info = json.loads(msg.body)
                    # Handle target logic using the received new target information
                    await self.handle_new_target(target_info["drone_id"], target_info["target"])
                elif performative == "confirm_charging_completion":
                    # Receive charging completion confirmation from Drone Agent
                    confirmation_info = json.loads(msg.body)
                    # Handle target reached logic using the received confirmation information
                    await self.handle_charging_completion_confirmation(confirmation_info["drone_id"])

        async def handle_new_target(self, drone_id, target):
            # save target and handle logic to send it there
            return

        async def handle_charging_completion_confirmation(self, drone_id):
            # assign drone to a land pad and send it there
            return

    async def setup(self):
        await super().setup()
        start_at = datetime.datetime.now() + datetime.timedelta(seconds=5)
        self.add_behaviour(self.DroneHandlingBehaviour())