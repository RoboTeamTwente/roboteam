# The environment

import gymnasium
from gymnasium import spaces
import numpy as np
from google.protobuf.message import DecodeError


# Import our functions
from sentActionCommand import send_action_command
from getState import get_ball_state, get_robot_state
from getRefereeState import get_referee_state
from teleportBall import teleport_ball
from resetRefereeState import reset_referee_state

class RoboTeamEnv(gymnasium.env):

    def __init__(self):

        self.MAX_ROBOTS = 10

        # Define the number of robots that are present in each grid + ball location
        self.robot_grid = np.zeros((4, 2), dtype=int)
        self.ball_location = np.zeros((1,2)) # Single row with 2 columns

        # Initialize the observation space
        self.observation_space = spaces.Dict({
            'robot_positions': spaces.Box(
                low=0,
                high=self.MAX_ROBOTS,
                shape=(4, 2),
                dtype=np.int32
            ),
            'ball_position': spaces.Discrete(5)  # 0-3 for quadrants, 4 for center
        })
        
        # Action space: [attackers, defenders]
        # Wallers will be automatically calculated
        self.action_space = spaces.MultiDiscrete([self.MAX_ROBOTS + 1, self.MAX_ROBOTS + 1])

        # Define the xy coordinates of the designated ball position
        self.x = 0 
        self.y = 0

        # Define yellow cards and current ref command
        self.yellow_yellow_cards = 0
        self.blue_yellow_cards = 0
        self.ref_command = "" # Empty string

    def teleport_ball_for_ball_placement(self):
        """
        Function to teleport the ball to the designated position for ball placement
        """

        # If ref gives command BALL_PLACEMENT_US OR BALL_PLACEMENT_THEM
        if (self.ref_command == "BALL_PLACEMENT_US") or ("BALL_PLACEMENT_THEM"):
            
            # Get the designated position
            teleport_ball(self.x, self.y)


    def get_referee_state(self):
        """
        Function to globally import the referee state
        """
        self.x,self.y, self.yellow_yellow_cards, self.blue_yellow_cards, self.ref_command = get_referee_state()

    def calculate_reward(self):
        """
        calculate_reward calculates the reward the agent gets for an action it did.
        Based on if we have the ball and if they scored a goal.
        """

        yellow_card_punishment = 0
        goal_scored_reward = 0

        # When a goal is scored the ref command is HALT

        # If we score a goal, give reward
        if goal_scored == True:
            goal_scored_reward 

        # If it gets a yellow card/ three times a foul, punish and reset
        if self.yellow_yellow_cards or self.blue_yellow_cards >= 1:
            yellow_card_punishment = 1


        # Calculate final reward
        reward = goal_scored_reward - yellow_card_punishment

        



        return reward


    def get_observation(self):
        
        """
        get_observation is meant to get the observation space (kinda like the state)
        """

        # Get the referee state
        stage, command, x, y, yellow_score, blue_score = get_referee_state()

        # Get the robot grid representation
        self.robot_grid = get_robot_state()

        # Get the ball location
        self.ball_location = get_ball_state()

        observation_space = {
            'robot_positions': self.robot_grid,
            'ball_position': self.ball_location
        }

        return observation_space, self.calculate_reward()



    def step(self, action):
        """
        The step function is called in every loop the RL agent goes through. 
        It receives a state, reward and carries out an action
        """

        attackers, defenders = action
        wallers = self.MAX_ROBOTS - (attackers + defenders)

        # Ensure non-negative values and total of 10
        attackers = max(0, min(attackers, self.MAX_ROBOTS))
        defenders = max(0, min(defenders, self.MAX_ROBOTS - attackers))
        wallers = self.MAX_ROBOTS - (attackers + defenders)

        # Fills the global variables with the integers from the script getRobotState
        self.num_of_robots_yellow_left, self.num_of_robots_yellow_right, 
        self.num_of_robots_blue_left, self.num_of_robots_blue_right = get_robot_state()
        
        # Sends the action command over proto to legacy AI
        send_action_command(num_attacker=attackers, num_defender=defenders, num_waller= wallers)

        reward = self.calculate_reward()

        # Update observation_space
        observation_space = None

        done = False

        done = self.is_terminated()  # If task is completed (a goal was scored)
        truncated = self.is_truncated()  # Determine if the episode was truncated, too much time or a yellow card

        return observation_space, reward, done, truncated



    def is_terminated(self):

        """
        Activates when the task has been completed (or it failed because of opponent scoring a goal)
        """

        if 

        pass



    def is_truncated(self):
        """
        is_truncated is meant for ending prematurely
        """



        pass


    def reset(self, seed=None):
        """
        The reset function resets the environment when a game is ended
        """

        # Teleport ball to middle position
        teleport_ball(0,0)

        # Reset referee state
        reset_referee_state()










