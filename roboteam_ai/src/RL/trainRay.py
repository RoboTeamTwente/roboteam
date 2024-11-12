#!/usr/bin/env python3.8

from env2 import RoboTeamEnv  # Ensure this points to your environment
import ray
import numpy as np
from ray.rllib.algorithms.ppo import PPOConfig
from ray.rllib.algorithms.callbacks import DefaultCallbacks
import os
import time

class CustomEvalCallbacks(DefaultCallbacks):
    eval_env_creator = None
    eval_freq = 70
    n_eval_episodes = 3

    @classmethod
    def set_parameters(cls, eval_env_creator, eval_freq=70, n_eval_episodes=3):
        cls.eval_env_creator = eval_env_creator
        cls.eval_freq = eval_freq
        cls.n_eval_episodes = n_eval_episodes

    def on_train_result(self, *, algorithm, result: dict, **kwargs):
        # Use class variables
        if result["timesteps_total"] % self.eval_freq == 0:
            eval_rewards = []
            eval_env = self.eval_env_creator()
            for _ in range(self.n_eval_episodes):
                episode_reward = 0
                obs, _ = eval_env.reset()
                done = False
                while not done:
                    action = algorithm.compute_single_action(obs)
                    obs, reward, done,truncated, _ = eval_env.step(action)
                    time.sleep(0.25)
                    episode_reward += reward
                eval_rewards.append(episode_reward)

            mean_reward = np.mean(eval_rewards)
            std_reward = np.std(eval_rewards)

            # Save the model if it is the best so far
            if mean_reward > self.best_mean_reward:
                self.best_mean_reward = mean_reward
                checkpoint_dir = algorithm.save("models/best_model_PPO")
                print(f"New best model saved at {checkpoint_dir} with mean reward {mean_reward:.2f}")

            # Log evaluation metrics
            result['custom_metrics']["mean_eval_reward"] = mean_reward
            result['custom_metrics']["std_eval_reward"] = std_reward
            print(f"Evaluation at step {result['timesteps_total']}: Mean reward = {mean_reward:.2f} +/- {std_reward:.2f}")

def main():
    # Set parameters for CustomEvalCallbacks
    eval_env_creator = lambda _: RoboTeamEnv(env_config={})  # Modified to accept one argument
    CustomEvalCallbacks.set_parameters(eval_env_creator=eval_env_creator, eval_freq=20, n_eval_episodes=1)

    # Initialize Ray and configure PPO algorithm
    ray.init(ignore_reinit_error=True)
    config = PPOConfig()
    config.environment(RoboTeamEnv, env_config={})
    config.framework("torch")
    config.env_runners(num_env_runners=1)
    config.training(
        train_batch_size=2048,
        lr=0.0003,
        gamma=0.99,
    )
    config.sgd_minibatch_size = 64
    config.gae_lambda = 0.95
    config.clip_param = 0.2
    config.entropy_coeff = 0.0
    config.vf_loss_coeff = 0.5
    config.callbacks(CustomEvalCallbacks)

    trainer = config.build()

    total_timesteps = 5000
    train_batch_size = config.train_batch_size
    total_iterations = total_timesteps // train_batch_size

    for iteration in range(total_iterations):
        result = trainer.train()
        mean_reward = result['episode_reward_mean']
        print(f"Iteration {iteration + 1}: Mean reward = {mean_reward}")

    ray.shutdown()

if __name__ == '__main__':
    main()
