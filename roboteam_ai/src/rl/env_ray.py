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
from roboteam_ai.src.rl.src.sentActionCommand import send_action_command, send_num_attackers
from roboteam_ai.src.rl.src.getState import get_ball_state, get_robot_state, get_referee_state
from roboteam_ai.src.rl.src.teleportBall import teleport_ball
from roboteam_ai.src.rl.src.resetRefereeAPI import reset_referee_state
from roboteam_ai.src.rl.src.changeGameState import start_game
from roboteam_ai.src.rl.src.teleportRobot import teleport_robots

"""
This environment file is in the form of a gymnasium environment.
We are yellow and we play against blue.

Yellow cards do not stop the game, but maybe in the future it is nice to implement a punishment
"""

class RoboTeamEnv(gymnasium.Env):

    def __init__(self, config=None):
        self.config = config or {} # Config placeholder
        self.MAX_ROBOTS_US = 10

        self.ball_position = np.zeros(2) # Single row with 2 columns

        self.yellow_score = 0  # Initialize score to zero
        self.blue_score = 0  # Initialize blue score to zero

        # Initialize the observation space for:
        # 44 values (robot positions) + 1 (is_yellow_dribbling) + 2 (ball x,y)
        self.observation_space = spaces.Box(
            low=float('-inf'), 
            high=float('inf'), 
            shape=(47,),  # 44 + 1 + 2
            dtype=np.float64
        )

        # Action space: single discrete value 0-6 representing number of attackers
        self.action_space = spaces.Discrete(7)  # 0 to 6 attackers possible

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

        # Add previous dribbling state
        self.previous_yellow_dribbling = False
        self.step_taken = False

        self.last_step_time = 0
        self.min_step_interval = 1.5  # Minimum time between steps in seconds

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
        if not self.shaped_reward_given and self.is_yellow_dribbling:
            self.shaped_reward_given = True # Set it to true
            shaped_reward = 0.1
        
        # # If it gets a yellow card/ three times a foul, punish and reset
        # if self.yellow_yellow_cards or self.blue_yellow_cards >= 1:
        #     yellow_card_punishment = 1

        # Calculate final reward
        reward = goal_scored_reward + shaped_reward

        return reward
    
    def get_observation(self):
        """
        Get observation space: robot positions (44) + dribbling (1) + ball position (2)
        """
        # Get the robot positions and dribbling states
        robot_positions, self.is_yellow_dribbling, self.is_blue_dribbling = get_robot_state()
        
        # Get the ball location (continuous x,y values)
        self.ball_position, self.ball_quadrant = get_ball_state()
        
        # Convert dribbling status to float64
        dribbling = np.array([float(self.is_yellow_dribbling)], dtype=np.float64)
        
        # Combine all parts into the observation array
        observation = np.concatenate([
            robot_positions,          # 44 elements (x,y for each robot)
            dribbling,               # 1 element
            self.ball_position       # 2 elements (x,y)
        ])
        
        return observation

    def step(self, action):
        """
        The step function waits for either:
        1. A true possession change (lost ball to opponent or gained from opponent)
        2. Specific referee commands (8,9)
        With rate limiting to prevent too frequent steps
        """
        
        while True:
            # Get current state
            observation_space = self.get_observation()
            
            # Get referee state
            self.yellow_score, self.blue_score, self.stage, self.ref_command, self.x, self.y = get_referee_state()

            if self.ref_command in (16,17): # If there is ball placement
                self.teleport_ball_with_check(self.x, self.y)

            # Reset step_taken flag if ref_command is 2
            if self.ref_command == 2:
                self.step_taken = False

            # Check for true possession change
            possession_changed = (
                (self.is_yellow_dribbling != self.previous_yellow_dribbling) and 
                (self.is_blue_dribbling or self.previous_yellow_dribbling)
            )

            # Check if enough time has passed since last step
            current_time = time.time()
            time_since_last_step = current_time - self.last_step_time
            can_take_step = time_since_last_step >= self.min_step_interval

            should_take_step = (
                can_take_step and (
                    possession_changed or 
                    (self.ref_command in (8,9) and not self.step_taken)
                )
            )

            if should_take_step:
                print(f"Taking step (time since last: {time_since_last_step:.2f}s)")
                # Update last step time
                self.last_step_time = current_time
                
                # Execute action
                send_num_attackers(action)
                
                # Mark step as taken for this ref state
                if self.ref_command in (8,9):
                    self.step_taken = True
                    
                # Update previous possession state
                self.previous_yellow_dribbling = self.is_yellow_dribbling
                
                reward = self.calculate_reward()
                
                # Check for termination
                truncated = self.is_truncated()
                done = self.is_terminated()
                
                return observation_space, reward, done, truncated, {}
            
            time.sleep(0.1)
                

    def is_terminated(self):
        """
        Activates when the task has been completed (or it failed because of opponent scoring a goal)
        """

        if self.ref_command == 0 and (self.yellow_score == 1 or self.blue_score == 1): # HALT command indicates that either team scored
            if self.yellow_score == 1:
                print("Yellow team scored")
            elif self.blue_score == 1:
                print("Blue team scored")
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
                print("Game truncated, goal wasn't accepted")
                return True
        if self.ref_command == 1:  # STOP
            print("Game truncated, random STOP called") 
            return True
        return False

    def reset(self, seed=None, options=None):
        """
        Reset the environment to initial state.
        """

        # Reset rate limiting variables
        self.last_step_time = time.time()  # Reset the timestamp

        # Reset internal state variables
        self.ball_position = np.zeros(2)
        self.yellow_score = 0
        self.blue_score = 0
        self.yellow_yellow_cards = 0
        self.blue_yellow_cards = 0
        self.ref_command = 0
        self.stage = 0
        self.shaped_reward_given = False
        self.is_yellow_dribbling = False
        self.is_blue_dribbling = False
        self.previous_yellow_dribbling = False  # Add this if not already in init
        self.step_taken = False  # Reset step taken flag

        # Reset physical environment
        print("Resetting physical environment...")
        
        # Ensure ball teleport completes
        self.teleport_ball_with_check(0,0)

        # Teleport the robots
        teleport_robots()

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
            observation = np.zeros(47, dtype=np.float64)

        # Set first_step flag
        self.is_first_step = True

        return observation, {}









