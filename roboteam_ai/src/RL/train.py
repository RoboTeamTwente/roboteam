import gymnasium as gym
from stable_baselines3 import PPO

# Import your custom environment
from env import RoboTeamEnv

# Create the environment
env = RoboTeamEnv()

# Create and train the PPO model
model = PPO("MultiInputPolicy", env, verbose=1)
model.learn(total_timesteps=1000)

# Test the trained model
obs, _ = env.reset()
for i in range(1000):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    env.render()
    if terminated or truncated:
        obs, _ = env.reset()

env.close()

