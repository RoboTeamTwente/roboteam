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
        self.observation_space = spaces.Box(low=float('-inf'), high=float('inf'), shape=(1,15), dtype=np.float64)

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
        referee_state, referee_info = get_referee_state()  # Assuming get_referee_state() is in scope

        # Extract the command from the referee state
        self.ref_command = referee_info['command']
        print("ref command", self.ref_command)
 

        # If ref gives command BALL_PLACEMENT_US OR BALL_PLACEMENT_THEM
        if (self.ref_command == 16 or self.ref_command == 17):
            referee_state, referee_data = get_referee_state()

            if referee_data["designated_position"]is not None:
                self.x, self.y = referee_data["designated_position"]["x"]/1000, referee_data["designated_position"]["y"]/1000

                # Teleport the ball to the designated location
                teleport_ball(self.x, self.y)
            else:
                print("No designated position provided in referee state.")


    def get_referee_state(self):
        """
        Function to globally import the referee state
        """
        self.x,self.y,  # Designated pos
        self.yellow_yellow_cards, self.blue_yellow_cards, # yellow cards
        self.ref_command, # Ref command, such as HALT, STOP
        self.yellow_score, self.blue_score = get_referee_state() # Scores

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
        print(f"Robot grid: {self.robot_grid}")
        print(f"Yellow dribbling: {self.is_yellow_dribbling}, Blue dribbling: {self.is_blue_dribbling}")

        # Get the ball location
        self.ball_position, self.ball_quadrant = get_ball_state()
        print(f"Ball position: {self.ball_position}, Ball quadrant: {self.ball_quadrant}")

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
        
        # Reshape to match expected shape (1, 15)
        observation = observation.reshape(1, 15)
        
        # Verify shape and dtype
        assert observation.shape == (1, 15), f"Observation shape {observation.shape} != (1, 15)"
        assert observation.dtype == np.float64, f"Observation dtype {observation.dtype} != float64"
        
        return observation, self.calculate_reward()

    def step(self, action):
        """
        The step function is called in every loop the RL agent goes through. 
        It receives a state, reward and carries out an action
        """

        # Only carry out "normal" loop if the game state is NORMAL_START (this indicates normal gameplay loop)
        if self.ref_command == "RUNNING": # Maybe this needs to change to normal_start

            attackers, defenders = action
            wallers = self.MAX_ROBOTS - (attackers + defenders)

            # Ensure non-negative values and total of 10
            attackers = max(0, min(attackers, self.MAX_ROBOTS))
            defenders = max(0, min(defenders, self.MAX_ROBOTS - attackers))
            wallers = self.MAX_ROBOTS - (attackers + defenders)

            # Sends the action command over proto to legacy AI
            send_action_command(num_attacker=attackers, num_defender=defenders, num_waller= wallers)


        # If the game is halted, stopped or ball placement is happening, execute this.

        # Logic to TP the ball if there is ball placement of either side
        self.check_ball_placement() # Run the function to check if we need to TP the ball

        reward = self.calculate_reward()

        # Update observation_space
        observation_space,_ = self.get_observation()
        
        done = self.is_terminated()
        print("isDone",done)  # If task is completed (a goal was scored)
        if done:
            observation_space, _ = self.reset()
        truncated = self.is_truncated()  # Determine if the episode was truncated, too much time or a yellow card

        time.sleep(0.1)  # DELAY FOR STEPS (ADJUST LATER)

        return observation_space, reward, done, truncated, {}


    def is_terminated(self):
        """
        Activates when the task has been completed (or it failed because of opponent scoring a goal)
        """
        referee_state, referee_info = get_referee_state()  # Assuming get_referee_state() is in scope

        self.ref_command = referee_info['command']
        self.yellow_score = referee_state.yellow.score
        self.blue_score = referee_state.blue.score
        
        print("refcommand", self.ref_command)
        print("blue", self.blue_score)
        print("yellow", self.yellow_score)

        if self.ref_command == 0 and (self.yellow_score == 1 or self.blue_score == 1): # HALT command indicates that either team scored
            return True

    def is_truncated(self):
        """
        is_truncated is meant for ending prematurely. For example when the time is ended (5 min)
        """

        # Implement logic to reset the game if no goal is scored
        pass

    def reset(self, seed=None,**kwargs):
        """
        The reset function resets the environment when a game is ended
        """

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
        observation, _ = self.get_observation()

        # Reset shaped_reward_given boolean
        self.shaped_reward_given = False
        self.is_yellow_dribbling = False
        self.is_blue_dribbling = False

        print("Reset complete!")
        return observation,{}









