# The environment

import gymnasium
from gymnasium import spaces
import numpy as np
from google.protobuf.message import DecodeError
import time
import os
import sys

# Make root folder /roboteam
current_dir = os.path.dirname(os.path.abspath(__file__))
roboteam_path = os.path.abspath(os.path.join(current_dir, "../../.."))

# Add to sys.path
sys.path.append(roboteam_path)

# Now import the functions
from roboteam_ai.src.RL.src.sentActionCommand import send_action_command
from roboteam_ai.src.RL.src.getState import get_ball_state, get_robot_state, get_referee_state
from roboteam_ai.src.RL.src.teleportBall import teleport_ball
from roboteam_ai.src.RL.src.resetRefereeAPI import reset_referee_state
from roboteam_ai.src.RL.src.changeGameState import start_game

"""
This environment file is in the form of a gymnasium environment.
We are yellow and we play against blue.

Yellow cards do not stop the game, but maybe in the future it is nice to implement a punishment
"""

class RoboTeamEnv(gymnasium.Env):

    def __init__(self, config=None):
        self.config = config or {} # Config placeholder
        self.MAX_ROBOTS_US = 10

        # Define the number of robots that are present in each grid + ball location
        self.robot_grid = np.zeros((4, 2), dtype=int) # left up, right up, left down, right down
        self.ball_position = np.zeros(2) # Single row with 2 columns
        self.ball_quadrant = 4 # This refers to the center

        self.yellow_score = 0  # Initialize score to zero
        self.blue_score = 0  # Initialize blue score to zero

        # Initialize the observation space
        self.observation_space = spaces.Box(low=float('-inf'), high=float('inf'), shape=(15,), dtype=np.float64)

        # Action space: [attackers, defenders]
        # Wallers will be automatically calculated
        self.action_space = spaces.MultiDiscrete([self.MAX_ROBOTS_US + 1, self.MAX_ROBOTS_US + 1])

        # Define the xy coordinates of the designated ball position
        self.x = 0 
        self.y = 0

        # Define ref state variables
        self.yellow_yellow_cards = 0
        self.blue_yellow_cards = 0
        self.ref_command = 0 # 0 means HALT
        self.stage = 0
        self.yellow_score = 0 # Init score to zero

        # Reward shaping
        self.shaped_reward_given = False # A reward that is given once per episode
        self.is_yellow_dribbling = False
        self.is_blue_dribbling = False

        # Set first_step flag
        self.is_first_step = True

    def teleport_ball_with_check(self, x, y):
        """
        Verify that ball teleportation completes successfully by attempting multiple times
        and checking the final position.
        
        Returns:
            bool: True if teleport succeeded, False otherwise
        """
        max_attempts = 25
        for i in range(max_attempts):
            try:
                teleport_ball(x, y) # Teleports ball to input of the function
                time.sleep(0.5)  # Give more time for physics to settle
                
                # Verify ball position
                ball_pos, _ = get_ball_state()
                if np.allclose(ball_pos, [x, y], atol=0.1): # Checks the real ball position from get_ball_state with the input of the function
                    print(f"Ball teleport successful on attempt {i+1}")
                    time.sleep(1)
                    return
            except Exception as e:
                print(f"Ball teleport attempt {i+1} failed: {str(e)}")
            
            if i == max_attempts - 1:
                print("Warning: Ball teleport may not have succeeded, big error")

    def calculate_reward(self):
        """
        calculate_reward calculates the reward the agent gets for an action it did.
        Based on if we have the ball and if they scored a goal.
        """
        goal_scored_reward = 0

        # When a goal is scored the ref command is HALT

        # If we score a goal, give reward. If opponent scores, give negative reward.
        if self.yellow_score == 1:
            goal_scored_reward = 1
        elif self.blue_score == 1:
            goal_scored_reward = -1

        # Reward shaping
        shaped_reward = 0
        if not self.shaped_reward_given and self.is_yellow_dribbling and (self.ball_quadrant == 1 or self.ball_quadrant == 3):
            self.shaped_reward_given = True # Set it to true
            shaped_reward = 0.1
        
        # # If it gets a yellow card/ three times a foul, punish and reset
        # if self.yellow_yellow_cards or self.blue_yellow_cards >= 1:
        #     yellow_card_punishment = 1

        # Calculate final reward
        reward = goal_scored_reward + shaped_reward

        return reward


    # def get_observation(self):
    #     """
    #     get_observation is meant to get the observation space (kinda like the state)
    #     """

    #     # Get the robot grid representation
    #     self.robot_grid, self.is_yellow_dribbling, self.is_blue_dribbling = get_robot_state() # Matrix of 4 by 2 + 2 booleans
    #     print(f"Robot grid: {self.robot_grid}")
    #     print(f"Yellow dribbling: {self.is_yellow_dribbling}, Blue dribbling: {self.is_blue_dribbling}")

    #     # Get the ball location
    #     self.ball_position, self.ball_quadrant = get_ball_state() # x,y coordinates, quadrant

    #     print(f"Ball position: {self.ball_position}, Ball quadrant: {self.ball_quadrant}")

    #     observation_space = {
    #         'robot_positions': self.robot_grid,
    #         'ball_position': self.ball_quadrant,
    #         'is_yellow_dribbling' : self.is_yellow_dribbling
    #     }

    #     robot_positions_flat = self.robot_grid.flatten()  # Shape (8,)
    #     ball_position_flat = np.array([self.ball_quadrant])  # Shape (1,)
    #     dribbling_flat = np.array([self.is_yellow_dribbling])  # Shape (1,)

    #     # Combine all parts into a single flat array
    #     full_observation = np.concatenate([robot_positions_flat, ball_position_flat, dribbling_flat])

    #     # Pad or reshape `full_observation` to shape [32, 10]
    #     padded_observation = np.zeros(32 * 10)  # Initialize a zero-padded observation
    #     padded_observation[:full_observation.size] = full_observation  # Fill with actual data
    #     observation_2d = padded_observation.reshape(32, 10)  # Reshape to [32, 10]

    #     return observation_2d, self.calculate_reward()
    
    def get_observation(self):
        """
        get_observation is meant to get the observation space (kinda like the state)
        """
        # Get the robot grid representation
        self.robot_grid, self.is_yellow_dribbling, self.is_blue_dribbling = get_robot_state()
        
        # Get the ball location
        self.ball_position, self.ball_quadrant = get_ball_state()
        
        # Convert and flatten robot positions to float64
        robot_positions_flat = self.robot_grid.astype(np.float64).flatten()  # 8 elements
        
        # Use ball quadrant for observation
        ball_quadrant = np.array([float(self.ball_quadrant)], dtype=np.float64)  # 1 element
        
        # Convert dribbling status to float64
        is_yellow_dribbling = np.array([float(self.is_yellow_dribbling)], dtype=np.float64)  # 1 element
        
        # Combine all parts into the observation array with padding
        observation = np.concatenate([
            robot_positions_flat,    # 8 elements
            ball_quadrant,          # 1 element
            is_yellow_dribbling,    # 1 element
            np.zeros(5, dtype=np.float64)  # 5 elements to reach total of 15
        ])
        
        # Make sure it's flat
        observation = observation.reshape(15,)
        
        return observation

    def step(self, action):
        """
        The step function is called in every loop the RL agent goes through. 
        It receives a state, reward and carries out an action
        """

        while True:
            self.yellow_score, self.blue_score, self.stage, self.ref_command, self.x, self.y = get_referee_state()

            if self.ref_command in (2,8,9):  # NORMAL_START, DIRECT_FREE_YELLOW, DIRECT_FREE_BLUE
                if self.is_first_step:
                    print("First step after reset - waiting for kickoff to finish...")
                    time.sleep(5)

                print("Game is RUNNING, executing action")
                attackers, defenders = action
                wallers = self.MAX_ROBOTS_US - (attackers + defenders)
                send_action_command(num_attacker=attackers, num_defender=defenders, num_waller=wallers)
                break

            # If HALT, but a goal is scored, we need to reset so we break
            # If HALT, and goal is false, we truncate.
            # If STOP, we truncate

            elif self.ref_command in (0, 1):  # HALT or STOP
                if self.ref_command == 0 and (self.yellow_score > 0 or self.blue_score > 0):
                    print('Goal is scored, resetting game state')
                    break
                print("Game truncated due to false goal or random STOP")
                break

            elif self.ref_command in (16, 17):  # Ball placement
                self.teleport_ball_with_check(self.x, self.y)

        self.is_first_step = False
        observation_space = self.get_observation()
        reward = self.calculate_reward()

        truncated = self.is_truncated()
        done = self.is_terminated()

        time.sleep(0.5)

        return observation_space, reward, done, truncated, {}

    def is_terminated(self):
        """
        Activates when the task has been completed (or it failed because of opponent scoring a goal)
        """

        if self.ref_command == 0 and (self.yellow_score == 1 or self.blue_score == 1): # HALT command indicates that either team scored
            return True
        else:
            return False

    def is_truncated(self):
        """
        is_truncated checks if game should end prematurely:
        - On HALT command with no goals scored
        - On STOP command
        """
        if self.ref_command == 0:  # HALT
            if (self.yellow_score == 0 and self.blue_score == 0):
                return True
        if self.ref_command == 1:  # STOP
            return True
        return False

    def reset(self, seed=None, options=None):
        """
        Reset the environment to initial state.
        """

        # Reset internal state variables
        self.robot_grid = np.zeros((4, 2), dtype=int)
        self.ball_position = np.zeros(2)
        self.ball_quadrant = 4
        self.yellow_score = 0
        self.blue_score = 0
        self.yellow_yellow_cards = 0
        self.blue_yellow_cards = 0
        self.ref_command = 0
        self.stage = 0
        self.shaped_reward_given = False
        self.is_yellow_dribbling = False
        self.is_blue_dribbling = False

        # Reset physical environment
        print("Resetting physical environment...")
        
        # Ensure ball teleport completes
        self.teleport_ball_with_check(0,0)

        # Reset referee state with verification
        try:
            reset_referee_state()
            time.sleep(0.2)  # Wait for referee state to update
            
            # Verify referee state
            yellow_score, blue_score, stage, ref_command, _, _ = get_referee_state()
            if not (yellow_score == 0 and blue_score == 0):
                print("Warning: Score reset may not have succeeded")
                
        except Exception as e:
            print(f"Error resetting referee state: {str(e)}")

        # Start new game
        try:
            start_game()
            time.sleep(0.2)  # Wait for game to start
            
            # Verify game started
            _, _, _, ref_command, _, _ = get_referee_state()
            if ref_command == 0:  # Still HALT
                print("Warning: Game may not have started properly")
                
        except Exception as e:
            print(f"Error starting game: {str(e)}")

        # Get initial observation
        try:
            observation = self.get_observation()
        except Exception as e:
            print(f"Error getting initial observation: {str(e)}")
            # Provide fallback observation if needed
            observation = np.zeros(15, dtype=np.float64)

        # Set first_step flag
        self.is_first_step = True

        return observation, {}









