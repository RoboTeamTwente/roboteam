#!/usr/bin/env python3

"""
Currently not in use, might start using it to go to a different way of looping through environment,
right now completely GPT'ed, do not use.
"""


import numpy as np
from gymnasium import spaces
from ray.rllib.env.external_env import ExternalEnv
import time
import os
import sys
from google.protobuf.message import DecodeError

# Make root folder /roboteam
current_dir = os.path.dirname(os.path.abspath(__file__))
roboteam_path = os.path.abspath(os.path.join(current_dir, "../../.."))
sys.path.append(roboteam_path)

from roboteam_ai.src.RL.src.sentActionCommand import send_action_command
from roboteam_ai.src.RL.src.getState import get_ball_state, get_robot_state, get_referee_state
from roboteam_ai.src.RL.src.teleportBall import teleport_ball
from roboteam_ai.src.RL.src.resetRefereeAPI import reset_referee_state
from roboteam_ai.src.RL.src.changeGameState import start_game

class RoboTeamEnv(ExternalEnv):
    def __init__(self, config=None):
        # Initialize state variables
        self.MAX_ROBOTS_US = 10
        self.robot_grid = np.zeros((4, 2), dtype=int)
        self.ball_position = np.zeros((1,2))
        self.ball_quadrant = 4
        self.yellow_score = 0
        self.blue_score = 0
        self.x = 0
        self.y = 0
        self.yellow_yellow_cards = 0
        self.blue_yellow_cards = 0
        self.ref_command = ""
        self.shaped_reward_given = False
        self.is_yellow_dribbling = False
        self.is_blue_dribbling = False

        # Define spaces before calling parent init
        action_space = spaces.MultiDiscrete([self.MAX_ROBOTS_US + 1, self.MAX_ROBOTS_US + 1])
        observation_space = spaces.Box(
            low=float('-inf'),
            high=float('inf'),
            shape=(15,),
            dtype=np.float64
        )

        # Initialize ExternalEnv parent with the spaces
        super().__init__(action_space=action_space, observation_space=observation_space)

        # Add Gymnasium compatibility attributes
        self._spec = None
        self.metadata = {
            'render_modes': [],
            'render.modes': [],
            'render_fps': None
        }
        self.reward_range = (-float('inf'), float('inf'))

    @property
    def unwrapped(self):
        """Gymnasium compatibility: Return the base environment."""
        return self

    @property
    def spec(self):
        """Gymnasium compatibility: Return environment specification."""
        return self._spec

    @spec.setter
    def spec(self, value):
        """Gymnasium compatibility: Set environment specification."""
        self._spec = value

    def _reset_sim(self):
        """Reset the simulation state"""
        try:
            print("Teleporting ball...")
            teleport_ball(0, 0)
            print("Sent command to teleport ball to (0, 0, 0.0)")
            print("Resetting referee state...")
            reset_referee_state()
            print("Starting game...")
            start_game()
            
            self.shaped_reward_given = False
            self.is_yellow_dribbling = False
            self.is_blue_dribbling = False
            self.yellow_score = 0
            self.blue_score = 0
        except Exception as e:
            print(f"Error resetting simulation: {e}")

    def _get_observation(self):
        """Get the current observation from the simulator"""
        try:
            # Get robot and ball state
            self.robot_grid, self.is_yellow_dribbling, self.is_blue_dribbling = get_robot_state()
            self.ball_position, self.ball_quadrant = get_ball_state()
            
            # Convert and flatten robot positions
            robot_positions_flat = self.robot_grid.astype(np.float64).flatten()  # 8 elements
            ball_quadrant = np.array([float(self.ball_quadrant)], dtype=np.float64)  # 1 element
            is_yellow_dribbling = np.array([float(self.is_yellow_dribbling)], dtype=np.float64)  # 1 element
            
            # Combine all parts
            observation = np.concatenate([
                robot_positions_flat,    # 8 elements
                ball_quadrant,          # 1 element
                is_yellow_dribbling,    # 1 element
                np.zeros(5, dtype=np.float64)  # 5 elements padding
            ])
            
            return observation.reshape(15,)
        except Exception as e:
            print(f"Error getting observation: {e}")
            return np.zeros(15, dtype=np.float64)

    def _execute_action(self, action):
        """Execute the action in the simulator"""
        try:
            print(f"x {self.x}")
            print(f"y {self.y}")
            print(f"Stepping with action: {action}")
            
            attackers, defenders = action
            
            # Ensure non-negative values and total of MAX_ROBOTS_US
            attackers = max(0, min(attackers, self.MAX_ROBOTS_US))
            defenders = max(0, min(defenders, self.MAX_ROBOTS_US - attackers))
            wallers = self.MAX_ROBOTS_US - (attackers + defenders)

            # Send action to simulator
            send_action_command(
                num_attacker=attackers,
                num_defender=defenders,
                num_waller=wallers
            )
        except Exception as e:
            print(f"Error executing action: {e}")

    def _check_ball_placement(self):
        """Handle ball placement commands"""
        try:
            referee_state, referee_info = get_referee_state()
            self.ref_command = referee_info['command']

            if (self.ref_command == 16 or self.ref_command == 17):
                if referee_info["designated_position"] is not None:
                    self.x = referee_info["designated_position"]["x"]/1000
                    self.y = referee_info["designated_position"]["y"]/1000
                    teleport_ball(self.x, self.y)
        except Exception as e:
            print(f"Error in ball placement: {e}")

    def _calculate_reward(self):
        """Calculate the reward for the current state"""
        try:
            goal_scored_reward = 0
            if self.yellow_score == 1:
                goal_scored_reward = 1
            elif self.blue_score == 1:
                goal_scored_reward = -1

            shaped_reward = 0
            if not self.shaped_reward_given and self.is_yellow_dribbling and (
                self.ball_quadrant == 1 or self.ball_quadrant == 3):
                self.shaped_reward_given = True
                shaped_reward = 0.1

            return goal_scored_reward + shaped_reward
        except Exception as e:
            print(f"Error calculating reward: {e}")
            return 0.0

    def _is_terminated(self):
        """Check if episode should terminate"""
        try:
            referee_state, referee_info = get_referee_state()
            self.ref_command = referee_info['command']
            self.yellow_score = referee_state.yellow.score
            self.blue_score = referee_state.blue.score
            
            return (self.ref_command == 0 and 
                   (self.yellow_score == 1 or self.blue_score == 1))
        except Exception as e:
            print(f"Error checking termination: {e}")
            return False

    def run(self):
        """Main control loop for the external environment."""
        while True:
            try:
                # Start new episode
                episode_id = self.start_episode()
                self._reset_sim()
                observation = self._get_observation()
                
                while True:
                    # Get action from policy
                    action = self.get_action(episode_id, observation)
                    
                    # Handle ball placement
                    self._check_ball_placement()
                    
                    # Execute action if game is running
                    if self.ref_command == "RUNNING":
                        self._execute_action(action)
                    
                    # Calculate reward before getting next observation
                    reward = self._calculate_reward()
                    self.log_returns(episode_id, reward)
                    
                    # Get next observation
                    observation = self._get_observation()
                    
                    # Check if episode is done
                    if self._is_terminated():
                        self.end_episode(episode_id, observation)
                        break
                    
                    time.sleep(0.1)  # Control loop rate

            except Exception as e:
                print(f"Error in main loop: {e}")
                time.sleep(1)  # Wait before retrying