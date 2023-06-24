import datetime
import json

from common.base_agent import BaseAgent
from common.config import (AGENT_NAMES, HEIGHT_RANGE, NUM_DRONES, DroneStatus,
                           OrderStatus, XMPP_SERVER_URL)
from common.utils import calculate_distance
from spade.behaviour import CyclicBehaviour, PeriodicBehaviour
from spade.message import Message


class TrafficControlStationAgent(BaseAgent):
    def __init__(self, jid: str, password: str):
        super().__init__(jid, password)
        self.drones = {}  # Dictionary to store drone information and status
        self.orders = {}  # Dictionary to store order information and status
        self.pending_assignment_orders = []  # List to store pending assignment orders

        # Setup drones dictionary
        for i in range(1, NUM_DRONES + 1):
            drone_id = str(i)
            self.drones[drone_id] = {
                "status": DroneStatus.UNAVAILABLE.value,
                "location": None,
                # Assign a valid route height for each drone, adding 5 meters in between each drone, starting from the bottom value in range
                "route_height": max((HEIGHT_RANGE[0] + ((i - 1) * 5)), HEIGHT_RANGE[1]), 
                "max_weight": float(1 * i), # max weight allowed per drone, in kg (multiplied i to simulate different characteristics)
                "assigned_order": None
            }

    class OrderAssignmentBehaviour(PeriodicBehaviour):
        async def run(self):
            # Check if there are pending assignment orders and, if so, attempt assignment of oldest in list (first element)
            if self.agent.pending_assignment_orders:
                pending_order = self.agent.pending_assignment_orders.pop(0)
                await self.attempt_assign_order(pending_order)

        async def attempt_assign_order(self, pending_order):
            order_id = pending_order["order_id"]
            details = pending_order["details"]

            self.agent.agent_say(f"Attempting to assign order {order_id}...")

            # Find the best available drone (closest available drone to dispatcher location with enough weight capacity)
            dispatcher_location = details["dispatcher_location"]
            available_drones = [(drone_id, drone_info) for drone_id, drone_info in self.agent.drones.items()
                if drone_info["status"] == DroneStatus.AVAILABLE.value and drone_info["max_weight"] >= float(details["weight"])]
            if available_drones:
                best_drone = min(
                    available_drones,
                    key=lambda drone: calculate_distance(dispatcher_location, drone[1]["location"])
                )[0]

                # Assign the order to the best available drone
                self.agent.orders[order_id] = {
                    "details": details,
                    "status": OrderStatus.ASSIGNED.value,
                    "assigned_drone": best_drone
                }

                self.agent.agent_say(f"Order {order_id} assigned to drone: {best_drone}")

                # Send route instructions to the assigned drone
                route_instructions = {
                    "order_id": order_id,
                    "dispatcher_location": details["dispatcher_location"],
                    "customer_location": details["customer_location"],
                    "qrcode": details["qrcode"],
                    "route_height": self.agent.drones[best_drone]["route_height"]
                }
                await self.send_assigned_route_instructions(best_drone, route_instructions)

                # Update order status to "ON_THE_WAY_TO_DISPATCHER"
                self.agent.orders[order_id]["status"] = OrderStatus.ON_THE_WAY_TO_DISPATCHER.value
            else:
                self.agent.pending_assignment_orders.insert(0, pending_order)
                self.agent.agent_say(f'There are no drones available. Reattempt will occur in 5 seconds.')     

        async def send_assigned_route_instructions(self, drone_id, route_instructions):
            route_msg = Message(to=f'{AGENT_NAMES["DRONE"]}{drone_id}@{XMPP_SERVER_URL}')
            route_msg.set_metadata("performative", "inform_route_instructions")
            route_msg.body = json.dumps(route_instructions)
            await self.send(route_msg)
            self.agent.agent_say(f'Route instructions sent to drone {drone_id}.')
            
    class OrderHandlingBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=10)
            if msg:
                performative = msg.get_metadata("performative")
                if performative == "inform_order":
                    # Receive order details from App agent
                    order_details = json.loads(msg.body)
                    await self.save_new_order(order_details["order_id"], order_details["details"])
                elif performative == "inform_status":
                    # Receive order status report from Drone agent
                    order_status = json.loads(msg.body)
                    await self.update_order_status(order_status["order_id"], order_status["status"])

        async def save_new_order(self, order_id, details):
            self.agent.agent_say(f"Saved new order {order_id} in pending assignment orders list")

            # Save new order in pending assignment orders list
            self.agent.pending_assignment_orders.append({
                "order_id": order_id,
                "details": details
            })

        async def update_order_status(self, order_id, new_status):
            if order_id in self.agent.orders:
                # Update order status
                self.agent.orders[order_id]["status"] = OrderStatus(new_status).value
                self.agent.agent_say(f"Order {order_id} status updated: {OrderStatus(new_status).name}")

                # Update drone status when order is completed
                if new_status == OrderStatus.COMPLETED.value:
                    assigned_drone = self.agent.orders[order_id]["assigned_drone"]
                    # Set the drone as charging if it's currently occupied pending charging
                    if self.agent.drones[assigned_drone]["status"] == DroneStatus.OCCUPIED_PENDING_CHARGING.value:
                        self.agent.drones[assigned_drone]["status"] = DroneStatus.CHARGING.value
                    # else, set the drone as available
                    else:
                        self.agent.drones[assigned_drone]["status"] = DroneStatus.AVAILABLE.value

                # Report order status to App agent
                await self.report_order_status(order_id, new_status)

                # Set order as completed if drone arrived at customer (for now, Arrived at customer is the final state, because there is no qr code handling yet)
                if new_status == OrderStatus.ARRIVED_AT_CUSTOMER.value:
                    await self.update_order_status(order_id, OrderStatus.COMPLETED.value)

        async def report_order_status(self, order_id, status):
            status_msg = Message(to=f'{AGENT_NAMES["APP"]}@{XMPP_SERVER_URL}')
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
                # Set the drone as available if it's currently unavailable and the reported location is valid (for example, if it's the first reported location)
                if self.agent.drones[drone_id]["status"] == DroneStatus.UNAVAILABLE.value and new_location != None and self.agent.drones[drone_id]["location"] == None:
                    self.agent.drones[drone_id]["status"] = DroneStatus.AVAILABLE.value
                # Set the drone as unavailable if it's currently available and the reported location is invalid (for example, when an error occurs in the drone)
                elif self.agent.drones[drone_id]["status"] == DroneStatus.AVAILABLE.value and new_location == None:
                    self.agent.drones[drone_id]["status"] = DroneStatus.UNAVAILABLE.value

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
                # Set the drone status as occupied pending charging if currently occupied 
                if self.agent.drones[drone_id]["status"] == DroneStatus.OCCUPIED.value:
                    self.agent.drones[drone_id]["status"] = DroneStatus.OCCUPIED_PENDING_CHARGING.value
                # Set the drone status as charging if currently unoccupied 
                elif self.agent.drones[drone_id]["status"] == DroneStatus.AVAILABLE.value:
                    self.agent.drones[drone_id]["status"] = DroneStatus.CHARGING.value
                
                # Redirect the request to the Charging Control Station Agent
                await self.redirect_charging_request(drone_id, self.agent.drones[drone_id]["location"], self.agent.drones[drone_id]["route_height"])

        async def redirect_charging_request(self, drone_id, drone_location, route_height):
            charging_request_msg = Message(to=f'{AGENT_NAMES["CHARGING_CONTROL_STATION"]}@{XMPP_SERVER_URL}')
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
        start_at = datetime.datetime.now() + datetime.timedelta(seconds=5)
        self.add_behaviour(self.OrderAssignmentBehaviour(period=5, start_at=start_at))
        self.add_behaviour(self.OrderHandlingBehaviour())
        self.add_behaviour(self.DroneLocationHandlingBehaviour())
        self.add_behaviour(self.DroneChargingHandlingBehaviour())
