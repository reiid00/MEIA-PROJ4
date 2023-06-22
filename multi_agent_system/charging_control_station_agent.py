import json

from common.base_agent import BaseAgent
from common.config import (AGENT_NAMES, OPTIMUM_BATTERY_RANGE, XMPP_SERVER_URL, ChargingStationSpotStatus, ChargingStationStatus)
from common.utils import calculate_distance
from spade.behaviour import CyclicBehaviour
from spade.message import Message


class ChargingControlStationAgent(BaseAgent):
    def __init__(self, jid: str, password: str):
        super().__init__(jid, password)
        # Simulated existing charging stations
        self.charging_stations = [
            {
                "id": 1,
                "location": { "latitude": 40.51, "longitude": 10.55 },
                "status": ChargingStationStatus.AVAILABLE.value,
                "spots": [
                    { "id": 1, "status": ChargingStationSpotStatus.AVAILABLE.value, "charging_drone": None },
                    { "id": 2, "status": ChargingStationSpotStatus.AVAILABLE.value, "charging_drone": None },
                    { "id": 3, "status": ChargingStationSpotStatus.AVAILABLE.value, "charging_drone": None }
                ]
            }
        ]
        # Dictionary to store charging drones and their battery status
        self.charging_drones = {}

    class ChargingRequestHandlingBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=10)
            if msg:
                performative = msg.get_metadata("performative")
                if performative == "request_charging":
                    charging_request = json.loads(msg.body)
                    await self.assign_charging_request(charging_request["drone_id"], charging_request["drone_location"], charging_request["route_height"])

        async def assign_charging_request(self, drone_id, drone_location, route_height):
            self.agent.agent_say(f"Assigning charging request for drone {drone_id}...")

            charging_stations = self.agent.charging_stations
            charging_stations_with_spots = [station for station in charging_stations if
                                            any(spot["status"] == ChargingStationSpotStatus.AVAILABLE.value for spot in station["spots"])]
            if charging_stations_with_spots:
                closest_station = min(charging_stations_with_spots,
                                      key=lambda station: calculate_distance(drone_location, station["location"]))
                available_spots = [spot for spot in closest_station["spots"] if
                                   spot["status"] == ChargingStationSpotStatus.AVAILABLE.value]
                available_spots[0]["status"] = ChargingStationSpotStatus.OCCUPIED.value
                available_spots[0]["charging_drone"] = drone_id
                await self.send_charging_instructions(drone_id, closest_station, available_spots[0], route_height)
            else:
                await self.attempt_early_charging(drone_id, route_height)

        async def send_charging_instructions(self, drone_id, charging_station, spot, route_height):
            self.agent.agent_say(f'Charging request for drone {drone_id} assigned to station: {charging_station["id"]}, spot {spot["id"]}')
            charging_instructions = {
                "charging_station_id": charging_station["id"],
                "charging_station_location": charging_station["location"],
                "spot_id": spot["id"],
                "route_height": route_height
            }
            charging_instructions_msg = Message(to=f'{AGENT_NAMES["DRONE"]}{drone_id}@{XMPP_SERVER_URL}')
            charging_instructions_msg.set_metadata("performative", "inform_charging_instructions")
            charging_instructions_msg.body = json.dumps(charging_instructions)
            await self.send(charging_instructions_msg)
            self.agent.agent_say(f'Charging instructions sent to drone {drone_id}.')

        async def attempt_early_charging(self, drone_id, route_height):
            self.agent.agent_say(f'Attempting early charging for drone {drone_id}...')  
            charging_drones = self.agent.charging_drones
            drones_reaching_threshold = [drone for drone, battery in charging_drones.items() if battery >= OPTIMUM_BATTERY_RANGE[1]]
            if drones_reaching_threshold:
                drone_with_highest_battery = max(drones_reaching_threshold, key=lambda drone: charging_drones[drone])
                await self.notify_early_charging(drone_with_highest_battery)
                await self.reassign_charging_spot(drone_with_highest_battery, drone_id, route_height)
                del self.agent.charging_drones[drone_with_highest_battery]
            else:
                self.agent.agent_say(f'No charging spots available. Unable to fulfill charging request for drone {drone_id}.')

        async def notify_early_charging(self, drone_id):
            notify_msg = Message(to=f'{AGENT_NAMES["DRONE"]}{drone_id}@{XMPP_SERVER_URL}')
            notify_msg.set_metadata("performative", "notify_early_charging")
            notify_msg.body = json.dumps(True)
            await self.send(notify_msg)
            self.agent.agent_say(f'Early charging successful, selected and notified drone {drone_id}.')

        async def reassign_charging_spot(self, current_drone_id, new_drone_id, route_height):
            charging_stations = self.agent.charging_stations
            for station in charging_stations:
                for spot in station["spots"]:
                    if spot["charging_drone"] == current_drone_id:
                        spot["charging_drone"] = new_drone_id
                        await self.send_charging_instructions(new_drone_id, station, spot, route_height)
                        return

    class BatteryReportBehaviour(CyclicBehaviour):
        async def run(self):
            # Receive battery reports from drones
            msg = await self.receive(timeout=10)
            if msg:
                performative = msg.get_metadata("performative")
                if performative == "inform_battery":
                    battery_report = json.loads(msg.body)
                    self.update_drone_battery(battery_report["drone_id"], battery_report["battery_percentage"])

        def update_drone_battery(self, drone_id, battery_percentage):
            # Check if charging is complete
            if battery_percentage >= 100:
                if drone_id in self.agent.charging_drones:
                    # Delete drone from charging drones list
                    del self.agent.charging_drones[drone_id]
                    # Update charging station spot availability
                    found_spot = False
                    for charging_station in self.agent.charging_stations:
                        for spot in charging_station["spots"]:
                            if spot["charging_drone"] == drone_id:
                                spot["charging_drone"] = None
                                spot["status"] = ChargingStationSpotStatus.AVAILABLE.value
                                charging_station["status"] = ChargingStationStatus.AVAILABLE.value
                                found_spot = True
                                break
                        if found_spot:
                            break
            else:
                self.agent.charging_drones[drone_id] = battery_percentage

    async def setup(self):
        await super().setup()
        self.add_behaviour(self.ChargingRequestHandlingBehaviour())
        self.add_behaviour(self.BatteryReportBehaviour())
