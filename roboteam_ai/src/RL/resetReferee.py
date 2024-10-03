
import sys
import os
from google.protobuf.timestamp_pb2 import Timestamp
from google.protobuf.duration_pb2 import Duration

# Make sure to go back to the main roboteam directory
current_dir = os.path.dirname(os.path.abspath(__file__))
roboteam_path = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))

# Add to sys.path
sys.path.append(roboteam_path)

# Assuming you have generated Python classes from your .proto files
from roboteam_networking.proto_CI.ssl_gc_state_pb2 import State, GameState, Command
from roboteam_networking.proto_CI.ssl_gc_referee_message_pb2 import Referee
from roboteam_networking.proto_CI.ssl_gc_common_pb2 import Team


def reset_game_state_blue_kickoff():
    # Create a new State message
    new_state = State()

    # Set the GameState to KICKOFF for Blue team
    new_state.game_state.type = GameState.KICKOFF
    new_state.game_state.for_team = Team.BLUE

    # Set the current command to KICKOFF for Blue team
    new_state.command.type = Command.KICKOFF
    new_state.command.for_team = Team.BLUE

    # Set Blue team as the first kickoff team
    new_state.first_kickoff_team = Team.BLUE

    # Reset stage and timers
    new_state.stage = Referee.NORMAL_FIRST_HALF
    new_state.stage_time_elapsed.Clear()
    new_state.stage_time_left.Clear()
    new_state.match_time_start.GetCurrentTime()  # Set to current time

    # Reset team states
    for team, is_blue in [("Blue", True), ("Yellow", False)]:
        team_info = new_state.team_state[team]
        team_info.name = team
        team_info.goals = 0
        team_info.yellow_cards.clear()
        team_info.red_cards.clear()
        team_info.timeouts_left = 4  # Or appropriate number
        team_info.timeout_time_left.CopyFrom(Duration(seconds=300))  # 5 minutes
        team_info.ball_placement_failures = 0
        team_info.on_positive_half = is_blue  # Blue on positive half, Yellow on negative

    return new_state

# Usage
new_state_change = reset_game_state_blue_kickoff()
# Use this new_state_change to update your game controller state