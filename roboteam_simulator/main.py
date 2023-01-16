import os

from RTTSimEnv import RTTSimEnv
from stable_baselines3 import A2C

log_dir = "logs"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

models_dir = "models/A2C"

if not os.path.exists(models_dir):
    os.makedirs(models_dir)

env = RTTSimEnv()
env.reset()

model = A2C('MlpPolicy', env, verbose=1, tensorboard_log=log_dir)

TIME_STEPS = 100000
iters = 0
while True:
    iters += 1
    model.learn(total_timesteps=TIME_STEPS, reset_num_timesteps=False, tb_log_name=f"A2C_1")
    model.save(f"{models_dir}/{TIME_STEPS * iters}")
