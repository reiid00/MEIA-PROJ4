from spade.agent import Agent


class BaseAgent(Agent):
    def __init__(self, jid: str, password: str):
        super().__init__(jid, password)
    
    def agent_say(self, text):
        print(str(self.jid) + ":\n\t" + str(text) + "\n")
    
    async def setup(self):
        self.agent_say("Agent starting . . .")

# import time

# ag = BaseAgent('senderagent@localhost', "dwdaw123")
# future = ag.start()
# future.result() # wait for agents to be prepared

# # Add a delay to allow the agent to complete its setup
# time.sleep(10)
# while ag.is_alive():
#     time.sleep(1)

