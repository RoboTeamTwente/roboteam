# train.py
import ray
from ray.rllib.algorithms.ppo import PPO

# Initialize Ray (when running locally with port-forwarding)
ray.init(address="ray://localhost:10001")

# Configure the training
config = {
    "env": "RoboTeam-v0",
    "num_workers": 1,
    "framework": "torch",
    "train_batch_size": 4000,
    "lr": 0.0003,
    "gamma": 0.99,
    "lambda": 0.95,
    "entropy_coeff": 0.01,
}

# Create trainer
trainer = PPO(config=config)

# Training loop
for i in range(100):  # 100 iterations
    result = trainer.train()
    print(f"Iteration {i}: reward = {result['episode_reward_mean']}")
    
    # Optional: save checkpoint every 10 iterations
    if i % 10 == 0:
        checkpoint_dir = trainer.save()
        print(f"Checkpoint saved in directory: {checkpoint_dir}")


