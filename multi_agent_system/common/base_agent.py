from spade.agent import Agent


class BaseAgent(Agent):
    def __init__(self, jid: str, password: str):
        super().__init__(jid, password)
    
    def agent_say(self, text):
        print(str(self.jid) + ":\n\t" + str(text) + "\n")
    
    async def setup(self):
        self.agent_say("Agent starting . . .")
