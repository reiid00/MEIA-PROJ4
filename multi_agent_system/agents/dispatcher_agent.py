import json
from spade.behaviour import CyclicBehaviour
from common.BaseAgent import BaseAgent
from config import DISPATCHER_API_URL

class DispatcherAgent(BaseAgent):
    class OrderHandlingBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=10)
            if msg:
                performative = msg.get_metadata("performative")
                if performative == "inform_order":
                    # Receive order details from App agent
                    order_details = json.loads(msg.body)
                    self.send_order_to_dispatcher_api(order_details["order_id"], order_details["details"])

        def send_order_to_dispatcher_api(self, order_id, order_details):
            # Simulate sending order details to the Dispatcher API
            # In this example, we simulate by logging the order details
            self.agent.agent_say(f"Order sent to Dispatcher API: {DISPATCHER_API_URL}\nOrder {order_id}")

    async def setup(self):
        await super().setup()
        self.add_behaviour(self.OrderHandlingBehaviour())
