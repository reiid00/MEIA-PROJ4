import datetime
import json

import requests
from common.base_agent import BaseAgent
from common.config import AGENT_NAMES, APP_API_URL, XMPP_SERVER_URL
from spade.behaviour import CyclicBehaviour, PeriodicBehaviour
from spade.message import Message


class AppAgent(BaseAgent):
    class OrderRequestBehaviour(PeriodicBehaviour):
        async def run(self):
            # Check if there are pending orders to be processed
            pending_order = self.get_pending_order()
            if pending_order:
                await self.send_order_to_dispatcher(pending_order)
                await self.send_order_to_traffic_control_station(pending_order)
                order_id = pending_order["order_id"]
                self.agent.agent_say(f"Order {order_id} details sent to TrafficControlStationAgent and DispatcherAgent")
                
        def get_pending_order(self):
            response = requests.get(f'{APP_API_URL}/next_pending_order')
            if response.status_code == 200:
                pending_order = response.json().get('next_pending_order', [])
                if pending_order:
                    return pending_order
            return None

        async def send_order_to_dispatcher(self, order_details):
<<<<<<< HEAD
            order_msg = Message(to=f'{AGENT_NAMES["DISPATCHER"]}@{XMPP_SERVER_URL}')
=======
            order_msg = Message(to=f'{AGENT_NAMES["DISPATCHER"]}@192.168.1.91')
>>>>>>> 4ccfd5987fc7055259ae78f6c1a2d4b06bf1ab17
            order_msg.set_metadata("performative", "inform_order")
            order_msg.body = json.dumps(order_details)
            await self.send(order_msg)

        async def send_order_to_traffic_control_station(self, order_details):
<<<<<<< HEAD
            order_msg = Message(to=f'{AGENT_NAMES["TRAFFIC_CONTROL_STATION"]}@{XMPP_SERVER_URL}')
=======
            order_msg = Message(to=f'{AGENT_NAMES["TRAFFIC_CONTROL_STATION"]}@192.168.1.91')
>>>>>>> 4ccfd5987fc7055259ae78f6c1a2d4b06bf1ab17
            order_msg.set_metadata("performative", "inform_order")
            order_msg.body = json.dumps(order_details)
            await self.send(order_msg)

    class OrderStatusReportBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=10)
            if msg:
                performative = msg.get_metadata("performative")
                if performative == "inform_status":
                    order_status = json.loads(msg.body)
                    await self.send_order_status_report_to_app_api(order_status["order_id"], order_status["status"])

        async def send_order_status_report_to_app_api(self, order_id, status):
            try:
                response = requests.post(f'{APP_API_URL}/update_order_status', json={
                    "order_id": order_id,
                    "status": status
                })
                if response.status_code == 200:
                    self.agent.agent_say(f"Order status update sent to App API: \nOrder {order_id} - Status {status}")
                else:
                    self.agent.agent_say(f"Failed to send order status update to App API. Error: {response.text}")
            except requests.RequestException as e:
                self.agent.agent_say(f"Failed to send order status update to App API. Exception: {e}")

    async def setup(self):
        await super().setup()
        start_at = datetime.datetime.now() + datetime.timedelta(seconds=5)
        self.add_behaviour(self.OrderRequestBehaviour(period=5, start_at=start_at))
        self.add_behaviour(self.OrderStatusReportBehaviour())
