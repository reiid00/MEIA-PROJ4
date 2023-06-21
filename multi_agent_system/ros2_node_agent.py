import datetime
import json

from common.base_agent import BaseAgent
from common.config import (AGENT_NAMES, DroneTargetType)
from spade.behaviour import CyclicBehaviour, PeriodicBehaviour
from spade.message import Message


class ROS2NodeAgent(BaseAgent):
    def __init__(self, jid: str, password: str):
        super().__init__(jid, password)

    class DroneInfoReportingBehaviour(PeriodicBehaviour):
        async def run(self):
            # TBI call send location and battery for each drone when needed 
            return

        async def send_location(self, drone_id, new_location):
            # Send current location report to Drone Agent
            location_msg = Message(to=f'{AGENT_NAMES["DRONE"]}{drone_id}@localhost')
            location_msg.set_metadata("performative", "inform_location")
            location_msg.body = json.dumps({"location": new_location})
            await self.send(location_msg)

        async def send_battery(self, drone_id, new_battery_percentage):
            # Send current battery report to Drone Agent
            battery_msg = Message(to=f'{AGENT_NAMES["DRONE"]}{drone_id}@localhost')
            battery_msg.set_metadata("performative", "inform_battery")
            battery_msg.body = json.dumps({"battery_percentage": new_battery_percentage})
            await self.send(battery_msg)

        async def send_target_reached_confirmation(self, drone_id, target):
            # Send target reached confirmation report to Drone Agent
            target_reached_confirmation_msg = Message(to=f'{AGENT_NAMES["DRONE"]}{drone_id}@localhost')
            target_reached_confirmation_msg.set_metadata("performative", "confirm_target_reached")
            target_reached_confirmation_msg.body = json.dumps(True)
            await self.send(target_reached_confirmation_msg)

    class DroneInfoHandlingBehaviour(CyclicBehaviour):
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
        self.add_behaviour(self.DroneInfoReportingBehaviour(period=5, start_at=start_at))
        self.add_behaviour(self.TargetHandlingBehaviour())