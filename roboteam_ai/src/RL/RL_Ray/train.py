#!/usr/bin/env python3
from pprint import pprint
import ray
from ray.rllib.algorithms.ppo import PPOConfig
from ray.tune.registry import register_env
from ray.tune.logger import JsonLoggerCallback, CSVLoggerCallback
import os
import sys
from ray.tune.registry import register_env

# Add roboteam_ai to Python path
roboteam_ai_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
print(f"Adding to PYTHONPATH: {roboteam_ai_root}")  # Debug print
sys.path.append(roboteam_ai_root)

from roboteam_ai.src.RL.env2 import RoboTeamEnv

import warnings
warnings.filterwarnings('ignore', category=DeprecationWarning)

def verify_imports():
    import numpy
    import torch
    print(f"Local NumPy version: {numpy.__version__}")
    print(f"Local PyTorch version: {torch.__version__}")

def main():
    verify_imports()

    if not ray.is_initialized():
        ray.init(
            address=f"ray://192.168.49.2:31001",
            ignore_reinit_error=True,
            runtime_env={
                "env_vars": {
                    "NUMPY_EXPERIMENTAL_ARRAY_FUNCTION": "0",

                },
                # "pip": [
                #     "numpy==1.24.3",
                #     "pyzmq==26.2.0"
                # ]
            }
        )

    # ray.init()

    # We can set env_config here
    def env_creator(env_config):
        return RoboTeamEnv(env_config)  # This passes the config to your env
    
    # Register the environment
    register_env("RoboTeamEnv", env_creator)

    # Create list of callbacks
    callbacks = [
        JsonLoggerCallback(),
        CSVLoggerCallback(),
    ]

    config = (
        PPOConfig()
        .environment("RoboTeamEnv")
        .framework("torch")
        .resources(num_gpus=0)
        .env_runners(
            num_env_runners=1,
            num_envs_per_env_runner=1,
            sample_timeout_s=None
        )
#         .api_stack(
#             enable_rl_module_and_learner=True,
#             enable_env_runner_and_connector_v2=True
# )
        .debugging(
            log_level="DEBUG",
            seed=42
        )
        #.callbacks(callbacks)
        .evaluation(evaluation_interval=10)
    )

    print("Starting training...")
    algo = config.build()

    for i in range(10):
        print(f"\nStarting iteration {i}")
        result = algo.train()
        result.pop("config")
        print("\nTraining metrics:")
        print(f"Episode Reward Mean: {result.get('episode_reward_mean', 'N/A')}")
        print(f"Episode Length Mean: {result.get('episode_len_mean', 'N/A')}")
        print(f"Total Timesteps: {result.get('timesteps_total', 'N/A')}")
        pprint(result)

if __name__ == "__main__":
    main()


