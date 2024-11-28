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
        self.ball_position = np.zeros((1,2)) # Single row with 2 columns
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
        self.ref_command = "" # Empty string
        self.yellow_score = 0 # Init score to zero

        # Reward shaping
        self.shaped_reward_given = False # A reward that is given once per episode
        self.is_yellow_dribbling = False
        self.is_blue_dribbling = False

    def check_ball_placement(self):
        """
        Function to teleport the ball to the designated position for ball placement if necessary.
        """
        # Get the current referee state
        get_referee_state()
 
        # If ref gives command BALL_PLACEMENT_US OR BALL_PLACEMENT_THEM
        if (self.ref_command == 16 or self.ref_command == 17):

            if self.x and self.y is not None:

                # Teleport the ball to the designated location
                teleport_ball(self.x, self.y)
            else:
                print("No designated position provided in referee state.")

    def get_referee_state(self):
        """
        Function to get referee state values
        """
        self.yellow_score, self.blue_score, self.stage, self.ref_command, self.x, self.y = get_referee_state()
        self.x = self.x/1000
        self.y = self.y/1000

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

        # Only carry out "normal" loop if the game state is NORMAL_START (this indicates normal gameplay loop)
        print(f"Step called with action: {action}")
        
        # Get current referee state at start of step
        #referee_state, referee_info = get_referee_state()
        #print(f"Step - Current referee state: {referee_state}")

        # if self.ref_command == "RUNNING":
        #     print("Game is RUNNING, executing action")
        #     attackers, defenders = action
        #     wallers = self.MAX_ROBOTS_US - (attackers + defenders)
        #     send_action_command(num_attacker=attackers, num_defender=defenders, num_waller=wallers)
        # else:
        #     print(f"Game not RUNNING, current command: {self.ref_command}")

        # If the game is halted, stopped or ball placement is happening, execute this.

        # Logic to TP the ball if there is ball placement of either side
        #self.check_ball_placement() # Run the function to check if we need to TP the ball

        # Update observation_space
        observation_space = self.get_observation()

        # Get reward
        reward = self.calculate_reward()
        
        done = self.is_terminated()
        #print("isDone",done)  # If task is completed (a goal was scored)
        if done:
            observation_space, _ = self.reset()
        truncated = self.is_truncated()  # Determine if the episode was truncated, too much time or a yellow card

        #time.sleep(0.1)  # DELAY FOR STEPS (ADJUST LATER)

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
        is_truncated is meant for ending prematurely. For example when the time is ended (5 min)
        """

        # Implement logic to reset the game if no goal is scored
        return False

    def reset(self, seed=None,**kwargs):
        """
        The reset function resets the environment when a game is ended
        """

        print("Testing...")

        # Teleport ball to middle position
        print("Teleporting ball...")
        teleport_ball(0,0)

        # Reset referee state
        print("Resetting referee state...")
        reset_referee_state()

        # Set blue team on right side + initiates kickoff
        print("Starting game...")
        start_game()

        print("Getting observation...")
        observation = self.get_observation()

        # Reset shaped_reward_given boolean
        self.shaped_reward_given = False
        self.is_yellow_dribbling = False
        self.is_blue_dribbling = False

        print("Reset complete!")
        return observation, {}









