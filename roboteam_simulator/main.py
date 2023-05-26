from RTTSimEnv import RTTSimEnv
from networking import Publisher, Subscriber

env = RTTSimEnv()
pub = Publisher(env)
sub = Subscriber(env)

pub.start()
sub.start()

while True:
    env.update()
