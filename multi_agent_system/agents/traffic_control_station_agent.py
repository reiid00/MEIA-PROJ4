import json
from spade.behaviour import CyclicBehaviour
from spade.message import Message
from common.BaseAgent import BaseAgent
from config import AGENT_NAMES, NUM_DRONES, HEIGHT_RANGE, OrderStatus, DroneStatus
from utils import calculate_distance

class TrafficControlStationAgent(BaseAgent):
    def __init__(self, jid: str, password: str):
        super().__init__(jid, password)
        self.drones = {}  # Dictionary to store drone information and status
        self.orders = {}  # Dictionary to store order information and status

        # Set up drones dictionary
        for i in range(1, NUM_DRONES + 1):
            drone_id = str(i)
            self.drones[drone_id] = {
                "status": DroneStatus.UNAVAILABLE.value,
                "location": None,
                "route_height": HEIGHT_RANGE[0],
                "assigned_order": None
            }

    class OrderHandlingBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=10)
            if msg:
                performative = msg.get_metadata("performative")
                if performative == "inform_order":
                    # Receive order details from App agent
                    order_details = json.loads(msg.body)
                    await self.assign_order(order_details["order_id"], order_details["details"])
                elif performative == "inform_status":
                    # Receive order status report from Drone agent
                    order_status = json.loads(msg.body)
                    await self.update_order_status(order_status["order_id"], order_status["status"])

        async def assign_order(self, order_id, details):
            self.agent.agent_say(f"Assigning order {order_id}...")

            # Save order
            self.orders[order_id] = {
                "details": details,
                "status": OrderStatus.TO_BE_ASSIGNED.value
            }

            # Find the best available drone (closest to dispatcher location)
            dispatcher_location = details["dispatcher_location"]
            available_drones = [(drone_id, drone_info) for drone_id, drone_info in self.agent.drones.items()
                if drone_info["status"] == DroneStatus.AVAILABLE.value]
            if available_drones:
                closest_drone = min(
                    available_drones,
                    key=lambda drone: calculate_distance(dispatcher_location, drone[1]["location"])
                )[0]

                # Assign the order to the best available drone
                self.orders[order_id]["status"] = OrderStatus.ASSIGNED.value
                self.orders[order_id]["assigned_drone"] = closest_drone
                self.agent.agent_say(f"Order {order_id} assigned to drone: {closest_drone}")

                # Send route instructions to the assigned drone
                route_instructions = {
                    "order_id": order_id,
                    "dispatcher_location": details["dispatcher_location"],
                    "customer_location": details["customer_location"],
                    "qrcode": details["qrcode"],
                    "route_height": HEIGHT_RANGE[0]
                }
                await self.send_assigned_route_instructions(closest_drone, route_instructions)

                # Update order status to "ON_THE_WAY_TO_DISPATCHER"
                self.orders[order_id]["status"] = OrderStatus.ON_THE_WAY_TO_DISPATCHER.value
            else:
                self.agent.agent_say(f'No drones available. Unable to fulfill order {order_id}.')     

        async def send_assigned_route_instructions(self, drone_id, route_instructions):
            route_msg = Message(to=f'{AGENT_NAMES["DRONE"]}{drone_id}@localhost')
            route_msg.set_metadata("performative", "inform_route")
            route_msg.body = json.dumps(route_instructions)
            await self.send(route_msg)
            self.agent.agent_say(f'Route instructions sent to drone {drone_id}.')

        async def update_order_status(self, order_id, new_status):
            if order_id in self.agent.orders:
                # Update order status
                self.agent.orders[order_id]["status"] = OrderStatus(new_status)
                self.agent.agent_say(f"Order {order_id} status updated: {OrderStatus(new_status).name}")

                # Report order status to App agent
                await self.report_order_status(order_id, new_status)

        async def report_order_status(self, order_id, status):
            status_msg = Message(to=f'{AGENT_NAMES["APP"]}@localhost')
            status_msg.set_metadata("performative", "inform_status")
            status_msg.body = json.dumps({"order_id": order_id, "status": status})
            await self.send(status_msg)

    class DroneLocationHandlingBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=10)
            if msg:
                performative = msg.get_metadata("performative")
                if performative == "inform_location":
                    # Receive drone location report from Drone agent
                    drone_location = json.loads(msg.body)
                    self.update_drone_location(drone_location["drone_id"], drone_location["location"])

        def update_drone_location(self, drone_id, new_location):
            if drone_id in self.agent.drones:
                # Set the drone as available if it's the first reported location
                if self.agent.drones[drone_id]["status"] == DroneStatus.UNAVAILABLE.value and self.agent.drones[drone_id]["location"] == None:
                    self.agent.drones[drone_id]["status"] = DroneStatus.AVAILABLE.value

                # Update drone location
                self.agent.drones[drone_id]["location"] = new_location

    class DroneChargingHandlingBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=10)
            if msg:
                performative = msg.get_metadata("performative")
                if performative == "request_charging":
                    # Receive charging request from Drone Agent
                    charging_request = json.loads(msg.body)
                    await self.handle_charging_request(charging_request["drone_id"])
                elif performative == "confirm_charging_completion":
                    # Receive charging completion confirmation from Drone Agent
                    charging_completion_confirmation = json.loads(msg.body)
                    self.handle_charging_completion_confirmation(charging_completion_confirmation["drone_id"])

        async def handle_charging_request(self, drone_id):
            self.agent.agent_say(f"Received charging request for drone {drone_id}.")
            if drone_id in self.agent.drones:
                # Update the drone status
                self.agent.drones[drone_id]["status"] = DroneStatus.CHARGING.value
                # Redirect the request to the Charging Control Station Agent
                await self.redirect_charging_request(drone_id, self.agent.drones[drone_id]["location"], HEIGHT_RANGE[0])

        async def redirect_charging_request(self, drone_id, drone_location, route_height):
            charging_request_msg = Message(to=f'{AGENT_NAMES["CHARGING_CONTROL_STATION"]}@localhost')
            charging_request_msg.set_metadata("performative", "request_charging")
            charging_request_msg.body = json.dumps({"drone_id": drone_id, "drone_location": drone_location, "route_height": route_height})
            await self.send(charging_request_msg)
            self.agent.agent_say(f"Redirected charging request for drone {drone_id} to Charging Control Station Agent.")

        def handle_charging_completion_confirmation(self, drone_id):
            self.agent.agent_say(f"Received charging completion confirmation for drone {drone_id}.")
            if drone_id in self.agent.drones:
                # Update the drone status
                self.agent.drones[drone_id]["status"] = DroneStatus.AVAILABLE.value

    async def setup(self):
        await super().setup()
        self.add_behaviour(self.OrderHandlingBehaviour())
        self.add_behaviour(self.DroneLocationHandlingBehaviour())
        self.add_behaviour(self.DroneChargingHandlingBehaviour())