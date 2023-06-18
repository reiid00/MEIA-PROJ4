import json

from apis.order_manager import pending_orders, pending_orders_lock
from common.base_agent import BaseAgent
from common.config import AGENT_NAMES, APP_API_URL
from spade.behaviour import CyclicBehaviour
from spade.message import Message


class AppAgent(BaseAgent):
    class OrderRequestBehaviour(CyclicBehaviour):
        async def run(self):
            print("tes")
            # Check if there are pending orders to be processed
            pending_order = self.get_pending_order()
            if pending_order:
                await self.send_order_to_dispatcher(pending_order)
                await self.send_order_to_traffic_control_station(pending_order)
                self.agent.agent_say(f"Order details sent to TrafficControlStationAgent and DispatcherAgent:\n{pending_order}")
     
        def get_pending_order(self):
            with pending_orders_lock:
                if pending_orders:
                    # Remove the first pending order from the list and return it
                    pending_order = pending_orders.pop(0)
                    return pending_order
            return None

        async def send_order_to_dispatcher(self, order_details):
            order_msg = Message(to=f'{AGENT_NAMES["DISPATCHER"]}@localhost')
            order_msg.set_metadata("performative", "inform_order")
            order_msg.body = json.dumps(order_details)
            await self.send(order_msg)

        async def send_order_to_traffic_control_station(self, order_details):
            order_msg = Message(to=f'{AGENT_NAMES["TRAFFIC_CONTROL_STATION"]}@localhost')
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
            # Simulate sending order status report to the App API
            # In this example, we simulate by logging the order status
            self.agent.agent_say(f"Order status update sent to App API: {APP_API_URL}\nOrder {order_id} - Status {status}")

    async def setup(self):
        print("tes2")
        await super().setup()
        self.add_behaviour(self.OrderRequestBehaviour())
        self.add_behaviour(self.OrderStatusReportBehaviour())
