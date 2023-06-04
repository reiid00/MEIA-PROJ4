import json
from spade.behaviour import CyclicBehaviour
from spade.message import Message
from common.BaseAgent import BaseAgent
from config import AGENT_NAMES, APP_API_URL
from order_manager import pending_orders, pending_orders_lock

class AppAgent(BaseAgent):
    class OrderRequestBehaviour(CyclicBehaviour):
        async def run(self):
            # Check if there are pending orders to be processed
            pending_order = self.get_pending_order()
            if pending_order:
                self.send_order_to_dispatcher(pending_order)
                self.send_order_to_traffic_control_station(pending_order)
                self.agent.agent_say(f"Order details sent to TrafficControlStationAgent and DispatcherAgent:\n{pending_order}")
     
        def get_pending_order(self):
            with pending_orders_lock:
                if pending_orders:
                    # Remove the first pending order from the list and return it
                    pending_order = pending_orders.pop(0)
                    return pending_order
            return None

        def send_order_to_dispatcher(self, order_details):
            order_msg = Message(to=f'{AGENT_NAMES["DISPATCHER"]}@localhost')
            order_msg.set_metadata("performative", "inform")
            order_msg.body = json.dumps(order_details)
            self.send(order_msg)

        def send_order_to_traffic_control_station(self, order_details):
            order_msg = Message(to=f'{AGENT_NAMES["TRAFFIC_CONTROL_STATION"]}@localhost')
            order_msg.set_metadata("performative", "inform")
            order_msg.body = json.dumps(order_details)
            self.send(order_msg)

    class OrderStatusReportBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=10)
            if msg:
                performative = msg.get_metadata("performative")
                if performative == "inform_status":
                    order_status = json.loads(msg.body)
                    self.send_order_status_report_to_app_api(order_status)

        def send_order_status_report_to_app_api(self, order_status):
            # Simulate sending order status report to the App API
            # In this example, we simulate by logging the order status
            order_id = order_status["order_id"]
            status = order_status["status"]
            self.agent.agent_say(f"Order status to App API: {APP_API_URL}\nOrder {order_id} - Status {status}")

    async def setup(self):
        await super().setup()
        self.add_behaviour(self.OrderRequestBehaviour())
        self.add_behaviour(self.OrderStatusReportBehaviour())
