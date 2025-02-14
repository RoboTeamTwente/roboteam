#!/usr/bin/env python3
from pprint import pprint
import ray
from ray.rllib.algorithms.ppo import PPOConfig
from ray.tune.registry import register_env
from ray import tune
import os
import sys

# Add roboteam_ai to Python path
roboteam_ai_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
print(f"Adding to PYTHONPATH: {roboteam_ai_root}")  # Debug print
sys.path.append(roboteam_ai_root)

from roboteam_ai.src.rl.env_ray import RoboTeamEnv

import warnings
warnings.filterwarnings('ignore', category=DeprecationWarning)

def main():
    def env_creator(env_config):
        return RoboTeamEnv(env_config)
    
    # Register the environment
    register_env("RoboTeamEnv", env_creator)

    config = (
        PPOConfig()
        .environment("RoboTeamEnv")
        .framework("torch")
        .resources(num_gpus=0)
        .env_runners(
            num_env_runners=0,
            num_envs_per_env_runner=1, # If you use vectorized env, otherwise set to 1
            # rollout_fragment_length=16,
            sample_timeout_s=120,
            create_env_on_local_worker=True) # This makes sure that we don't run a local environment
        .api_stack(
            enable_rl_module_and_learner=True,
            enable_env_runner_and_connector_v2=True)
        .debugging(
            log_level="DEBUG",
            seed=42
        )
        .training(
            train_batch_size_per_learner=512,
        )
    )

    algo = config.build()
    for i in range(10):
        result = algo.train()
        result.pop("config")
        pprint(result)

    if i % 5 == 0:
        checkpoint_dir = algo.save_to_path()
        print(f"Checkpoint saved in directory {checkpoint_dir}")

if __name__ == "__main__":
    main()