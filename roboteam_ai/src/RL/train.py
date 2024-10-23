#!/usr/bin/env python3.8

from env import RoboTeamEnv
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, EvalCallback
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.monitor import Monitor
import numpy as np
import os

class CustomEvalCallback(BaseCallback):
    def __init__(self, eval_env, eval_freq=70, n_eval_episodes=3, verbose=1):
        super(CustomEvalCallback, self).__init__(verbose)
        self.eval_env = eval_env
        self.eval_freq = eval_freq
        self.n_eval_episodes = n_eval_episodes
        self.best_mean_reward = -np.inf
        self.highest_env_value = -np.inf
        
    def _on_step(self) -> bool:
        if self.n_calls % self.eval_freq == 0:
            mean_reward, std_reward = evaluate_policy(
                self.model,
                self.eval_env,
                n_eval_episodes=self.n_eval_episodes,
                deterministic=True
            )

            train_env_value = self.training_env.get_attr("reconstructed_surface_coverage")

            if mean_reward > self.best_mean_reward:
                self.best_mean_reward = mean_reward
                self.model.save("best_model_PPO")

            self.logger.record("eval/mean_reward", mean_reward)
            self.logger.record("eval/std_reward", std_reward)
            self.logger.record("eval/highest_value_reconstruction", self.highest_env_value)

            print(f"Evaluation at step {self.n_calls}: Mean reward = {mean_reward:.2f} +/- {std_reward:.2f}")
            print(f"Evaluation highest env value = {self.highest_env_value}")
            print(f"Training reconstruction value = {train_env_value}")

        return True

def main():
    log_dir = "tmp/"
    os.makedirs(log_dir, exist_ok=True)

    # Create the main environment
    env = RoboTeamEnv()
    env = Monitor(env, filename=os.path.join(log_dir, "monitor"))

    # Create the evaluation environment
    eval_env = RoboTeamEnv()
    eval_env = Monitor(eval_env, filename=os.path.join(log_dir, "eval_monitor"))

    print("Observation space:", env.observation_space)
    
    model = PPO("MultiInputPolicy", env, verbose=1, 
        learning_rate=0.0003,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.0,
        vf_coef=0.5,
        max_grad_norm=0.5,
        tensorboard_log="./ppo_tensorboard_main/")
    
    # Create the evaluation callback
    eval_callback = CustomEvalCallback(eval_env, eval_freq=20, n_eval_episodes=1)

    total_timesteps = 5000 # 5000

    model.learn(total_timesteps=total_timesteps, callback=[eval_callback])

if __name__ == '__main__':
    main()