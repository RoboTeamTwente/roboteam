from . websocketHandler import run_websocket_command

def reset_referee_state():
    reset_msg = {"reset_match": True}
    return run_websocket_command(reset_msg)