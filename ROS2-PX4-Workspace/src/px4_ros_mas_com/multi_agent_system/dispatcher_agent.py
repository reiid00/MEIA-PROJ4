import json

from common.base_agent import BaseAgent
from common.config import DISPATCHER_API_URL
from spade.behaviour import CyclicBehaviour


class DispatcherAgent(BaseAgent):
    class OrderHandlingBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=10)
            if msg:
                performative = msg.get_metadata("performative")
                if performative == "inform_order":
                    # Receive order details from App agent
                    order_details = json.loads(msg.body)
                    await self.send_order_to_dispatcher_api(order_details["order_id"], order_details["details"])

        async def send_order_to_dispatcher_api(self, order_id, details):
            # Simulate sending order details to the Dispatcher API
            # In this example, we simulate by logging the order details
            self.agent.agent_say(f"Order {order_id} sent to Dispatcher API: {DISPATCHER_API_URL}")

    async def setup(self):
        await super().setup()
        self.add_behaviour(self.OrderHandlingBehaviour())
