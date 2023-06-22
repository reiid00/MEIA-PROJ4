import asyncio
import datetime
import json

from .base_agent import BaseAgent
from .config import (AGENT_NAMES, OPTIMUM_BATTERY_RANGE, DroneStatus,
                           DroneTargetType, OrderStatus)
from spade.behaviour import CyclicBehaviour, PeriodicBehaviour
from spade.message import Message


class DroneAgent(BaseAgent):
    def __init__(self, jid: str, password: str, drone_id: str):
        super().__init__(jid, password)
        self.drone_id = drone_id
        self.status = DroneStatus.UNAVAILABLE.value
        self.location = None
        self.targets = [] # Should follow model { type: DroneTargetType, location: { latitude, longitude }, route_height, order_id (optional), spot_id (optional)}
        self.battery_percentage = 100

    class DroneInfoHandlingBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=10)
            if msg:
                performative = msg.get_metadata("performative")
                if performative == "inform_location":
                    # Receive drone location report from ROS2 node agent
                    drone_location = json.loads(msg.body)
                    self.location = drone_location["location"]
                elif performative == "inform_battery":
                    # Receive drone battery report from ROS2 node agent
                    drone_battery = json.loads(msg.body)
                    self.battery_percentage = drone_battery["battery_percentage"]

    class LocationReportingBehaviour(PeriodicBehaviour):
        async def run(self):
            # Send current location report to Traffic Control Station Agent
            location_msg = Message(to=f'{AGENT_NAMES["TRAFFIC_CONTROL_STATION"]}@192.168.1.91')
            location_msg.set_metadata("performative", "inform_location")
            location_msg.body = json.dumps({"drone_id": self.agent.drone_id, "location": self.agent.location})
            self.agent.agent_say(f'LOCATION SENT: {location_msg.body}')
            await self.send(location_msg)

    class BatteryHandlingBehaviour(PeriodicBehaviour):
        async def run(self):
            # Check if currently charging
            if self.agent.status == DroneStatus.CHARGING.value:
                # Check if charging is complete
                if self.agent.battery_percentage >= 100:
                    # Set drone status as available after completing charging
                    self.agent.status = DroneStatus.AVAILABLE.value
                    # Notify traffic control station agent of charging completion
                    await self.send_charging_completion_confirmation_to_traffic_control_station()
                    # Notify ROS2 node agent of charging completion
                    await self.send_charging_completion_confirmation_to_ros2_node()
                    self.agent.agent_say(f"Charging completion confirmation sent to TrafficControlStationAgent and ROS2NodeAgent")

                # Send current battery report to Charging Control Station Agent
                battery_msg = Message(to=f'{AGENT_NAMES["CHARGING_CONTROL_STATION"]}@192.168.1.91')
                battery_msg.set_metadata("performative", "inform_battery")
                battery_msg.body = json.dumps({"drone_id": self.agent.drone_id, "battery_percentage": self.agent.battery_percentage})
                await self.send(battery_msg)
            
            # Check if needs charging
            if self.agent.status == DroneStatus.AVAILABLE.value or self.agent.status == DroneStatus.OCCUPIED.value:
                # Check if charging is required based on battery level
                if self.agent.battery_percentage < OPTIMUM_BATTERY_RANGE[0]:
                    if self.agent.status != DroneStatus.OCCUPIED.value:
                        self.agent.status = DroneStatus.PENDING_CHARGING.value
                    else:
                        self.agent.status = DroneStatus.OCCUPIED_PENDING_CHARGING.value
                    # Request charging spot from Traffic Control Station Agent
                    await self.request_charging()

        async def request_charging(self):
            charging_request_msg = Message(to=f'{AGENT_NAMES["TRAFFIC_CONTROL_STATION"]}@192.168.1.91')
            charging_request_msg.set_metadata("performative", "request_charging")
            charging_request_msg.body = json.dumps({"drone_id": self.agent.drone_id})
            await self.send(charging_request_msg)
            self.agent.agent_say(f'Battery level is low. Requesting charging.')

        async def send_charging_completion_confirmation_to_traffic_control_station(self):
            confirmation_msg = Message(to=f'{AGENT_NAMES["TRAFFIC_CONTROL_STATION"]}@192.168.1.91')
            confirmation_msg.set_metadata("performative", "confirm_charging_completion")
            confirmation_msg.body = json.dumps({"drone_id": self.agent.drone_id})
            await self.send(confirmation_msg)

        async def send_charging_completion_confirmation_to_ros2_node(self):
            confirmation_msg = Message(to=f'{AGENT_NAMES["ROS2_NODE"]}@192.168.1.91')
            confirmation_msg.set_metadata("performative", "confirm_charging_completion")
            confirmation_msg.body = json.dumps({"drone_id": self.agent.drone_id})
            await self.send(confirmation_msg)

    class ChargingHandlingBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=10)
            if msg:
                performative = msg.get_metadata("performative")
                if performative == "inform_charging_instructions":
                    # Receive charging instructions from Charging Control Station Agent
                    charging_instructions = json.loads(msg.body)
                    # Handle charging logic using the received instructions
                    await self.handle_charging_instructions(charging_instructions)
                elif performative == "notify_early_charging":
                    # Receive early charging notification from Charging Control Station Agent
                    notification = json.loads(msg.body)
                    # Handle early charging logic using the received notification
                    await self.handle_early_charging_notification(notification)

        async def handle_charging_instructions(self, charging_instructions):
            charging_station_target = {
                "type": DroneTargetType.CHARGING_STATION.value,
                "location": charging_instructions["charging_station_location"],
                "route_height": charging_instructions["route_height"],
                "charging_station_id": charging_instructions["charging_station_id"],
                "spot_id": charging_instructions["spot_id"]
            }

            # If targets list is empty, then should send charging station target immediately
            send_charging_station_target = True if not self.agent.targets else False

            # Add charging station location as a target location
            self.agent.targets.append(charging_station_target)
            self.agent.agent_say(f'Received charging instructions. Added charging station {charging_instructions["charging_station_id"]} location to targets list.')
            
            # Send charging station target to ROS2 node agent
            if send_charging_station_target:
                # Update the drone status
                self.agent.status = DroneStatus.ON_THE_WAY_TO_CHARGING_STATION.value
                # Send target
                await self.send_target(charging_station_target)

        async def send_target(self, target):
            target_msg = Message(to=f'{AGENT_NAMES["ROS2_NODE"]}@192.168.1.91')
            target_msg.set_metadata("performative", "inform_target")
            target_msg.body = json.dumps({"drone_id": self.agent.drone_id, "target": target})
            await self.send(target_msg)
            self.agent.agent_say(f'Sent target: {target["type"].name}')

        async def handle_early_charging_notification(self, notification):
            if notification:
                # Set drone status as available after completing early charging
                self.agent.status = DroneStatus.AVAILABLE.value
                # Notify traffic control station agent of charging completion
                await self.send_charging_completion_confirmation_to_traffic_control_station()
                # Notify ROS2 node agent of charging completion
                await self.send_charging_completion_confirmation_to_ros2_node()
                self.agent.agent_say(f"Charging completion confirmation sent to TrafficControlStationAgent and ROS2NodeAgent")

        async def send_charging_completion_confirmation_to_traffic_control_station(self):
            confirmation_msg = Message(to=f'{AGENT_NAMES["TRAFFIC_CONTROL_STATION"]}@192.168.1.91')
            confirmation_msg.set_metadata("performative", "confirm_charging_completion")
            confirmation_msg.body = json.dumps({"drone_id": self.agent.drone_id})
            await self.send(confirmation_msg)

        async def send_charging_completion_confirmation_to_ros2_node(self):
            confirmation_msg = Message(to=f'{AGENT_NAMES["ROS2_NODE"]}@192.168.1.91')
            confirmation_msg.set_metadata("performative", "confirm_charging_completion")
            confirmation_msg.body = json.dumps({"drone_id": self.agent.drone_id})
            await self.send(confirmation_msg)

    class TargetReachedHandlingBehaviour(CyclicBehaviour):
        async def run(self):
            # Wait for an event or trigger
            # You can use any suitable mechanism to wait for the event
            await self.wait_for_event()

            # Once the event is triggered, send a communication
            await self.handle_target_reached_confirmation()

        async def wait_for_event(self):
            # Implement the logic to wait for the event
            # This could be based on a condition, a callback, a message received, etc.
            # You can use asyncio primitives like events or queues, or any other mechanism

            # Example using asyncio event:
            event = asyncio.Event()
            # Wait for the event to be set or triggered
            await event.wait()

        async def handle_target_reached_confirmation(self):
            # Remove the current target from the targets list
            current_target = self.agent.targets.pop(0)
            self.agent.agent_say(f'Target reached: {current_target["type"].name}')

            if current_target["type"] == DroneTargetType.DISPATCHER.value:
                # Send order status update to Traffic Control Station Agent
                self.send_order_status_update(current_target["order_id"], OrderStatus.ARRIVED_AT_DISPATCHER.value)
            elif current_target["type"] == DroneTargetType.CUSTOMER.value:
                # Send order status update to Traffic Control Station Agent
                self.send_order_status_update(current_target["order_id"], OrderStatus.ARRIVED_AT_CUSTOMER.value)
            elif current_target["type"] == DroneTargetType.CHARGING_STATION.value:
                # Update the drone status
                self.agent.status = DroneStatus.CHARGING.value

            # Send next target, if exists, to ROS2 node agent
            if self.agent.targets:
                await self.send_target(self.agent.targets[0])

        async def send_target(self, target):
            if target["type"] == DroneTargetType.DISPATCHER.value:
                # Send order status update to Traffic Control Station Agent
                self.send_order_status_update(target["order_id"], OrderStatus.ON_THE_WAY_TO_DISPATCHER.value)
            elif target["type"] == DroneTargetType.CUSTOMER.value:
                # Send order status update to Traffic Control Station Agent
                self.send_order_status_update(target["order_id"], OrderStatus.ON_THE_WAY_TO_CUSTOMER.value)
            elif target["type"] == DroneTargetType.CHARGING_STATION.value:
                # Update the drone status
                self.agent.status = DroneStatus.ON_THE_WAY_TO_CHARGING_STATION.value

            target_msg = Message(to=f'{AGENT_NAMES["ROS2_NODE"]}@192.168.1.91')
            target_msg.set_metadata("performative", "inform_target")
            target_msg.body = json.dumps({"drone_id": self.agent.drone_id, "target": target})
            await self.send(target_msg)
            self.agent.agent_say(f'Sent target: {target["type"].name}')

    class RouteHandlingBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=10)
            if msg:
                performative = msg.get_metadata("performative")
                if performative == "inform_route_instructions":
                    # Receive route instructions from Traffic Control Station Agent
                    route_instructions = json.loads(msg.body)
                    # Handle route logic using the received instructions
                    await self.handle_route_instructions(route_instructions)

        async def handle_route_instructions(self, route_instructions):
            dispatcher_target = {
                "type": DroneTargetType.DISPATCHER.value,
                "location": route_instructions["dispatcher_location"],
                "route_height": route_instructions["route_height"],
            }
            customer_target = {
                "type": DroneTargetType.CUSTOMER.value,
                "location": route_instructions["customer_location"],
                "route_height": route_instructions["route_height"],
                "qrcode": route_instructions["qrcode"]
            }

            # If targets list is empty, then should send dispatcher target immediately
            send_dispatcher_target = True if not self.agent.targets else False
            
            # Add dispatcher and customer locations as target locations
            self.agent.targets.append(dispatcher_target)
            self.agent.targets.append(customer_target)
            self.agent.agent_say(f'Received route instructions. Added dispatcher and customer locations of order {route_instructions["order_id"]} to targets list.')
            
            if send_dispatcher_target:
                # Update the drone status
                self.agent.status = DroneStatus.OCCUPIED.value
                # Send dispatcher target
                await self.send_target(dispatcher_target)

        async def send_order_status_update(self, order_id, status):
            status_msg = Message(to=f'{AGENT_NAMES["TRAFFIC_CONTROL_STATION"]}@192.168.1.91')
            status_msg.set_metadata("performative", "inform_status")
            status_msg.body = json.dumps({"drone_id": self.agent.drone_id, "order_id": order_id, "status": status})
            await self.send(status_msg)
            self.agent.agent_say(f'Sent order status update: Order {order_id} - Status {status}')

        async def send_target(self, target):
            if target["type"] == DroneTargetType.DISPATCHER.value:
                # Send order status update to Traffic Control Station Agent
                self.send_order_status_update(target["order_id"], OrderStatus.ON_THE_WAY_TO_DISPATCHER.value)
            elif target["type"] == DroneTargetType.CUSTOMER.value:
                # Send order status update to Traffic Control Station Agent
                self.send_order_status_update(target["order_id"], OrderStatus.ON_THE_WAY_TO_CUSTOMER.value)
            elif target["type"] == DroneTargetType.CHARGING_STATION.value:
                # Update the drone status
                self.agent.status = DroneStatus.ON_THE_WAY_TO_CHARGING_STATION.value

            target_msg = Message(to=f'{AGENT_NAMES["ROS2_NODE"]}@192.168.1.91')
            target_msg.set_metadata("performative", "inform_target")
            target_msg.body = json.dumps({"drone_id": self.agent.drone_id, "target": target})
            await self.send(target_msg)
            self.agent.agent_say(f'Sent target: {target["type"].name}')

    async def setup(self):
        await super().setup()
        start_at = datetime.datetime.now() + datetime.timedelta(seconds=5)
        self.add_behaviour(self.LocationReportingBehaviour(period=5, start_at=start_at))
        self.add_behaviour(self.BatteryHandlingBehaviour(period=5, start_at=start_at))
        self.add_behaviour(self.TargetReachedHandlingBehaviour())
        self.add_behaviour(self.DroneInfoHandlingBehaviour())
        self.add_behaviour(self.ChargingHandlingBehaviour())
        self.add_behaviour(self.RouteHandlingBehaviour())