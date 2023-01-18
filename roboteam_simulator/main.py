from RTTSimEnv import RTTSimEnv
from networking import Publisher, Subscriber
import threading

env = RTTSimEnv()
pub = Publisher()
sub = Subscriber()

done = False
while not done:
    env.reset()
    pub.send(env.field)
    sub.receive()
    obs, reward, done, info = env.step(env.action_space.sample())
