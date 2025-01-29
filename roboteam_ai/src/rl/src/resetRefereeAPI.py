import os
import sys

# Make sure to go back to the main roboteam directory
current_dir = os.path.dirname(os.path.abspath(__file__))
roboteam_path = os.path.abspath(os.path.join(current_dir, "..","..", "..", ".."))

# Add to sys.path
sys.path.append(roboteam_path)

from roboteam_ai.src.rl.src.websocketHandler import run_websocket_command

def reset_referee_state():
    reset_msg = {"reset_match": True}
    return run_websocket_command(reset_msg)


reset_referee_state()