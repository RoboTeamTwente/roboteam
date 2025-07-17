from . websocketHandler import run_websocket_command

def set_team_state(team, on_positive_half):
    state_msg = {
        "change": {
            "update_team_state_change": {
                "for_team": team,
                "on_positive_half": on_positive_half
            }
        }
    }
    return run_websocket_command(state_msg)

def send_referee_command(command_type, team=None):
    command_msg = {
        "change": {
            "new_command_change": {
                "command": {
                    "type": command_type,
                    "for_team": team if team else "UNKNOWN"
                }
            }
        }
    }
    return run_websocket_command(command_msg)

def set_first_kickoff_team(team):
    """Set which team takes first kickoff."""
    config_msg = {
        "change": {
            "update_config_change": {
                "first_kickoff_team": team
            }
        }
    }
    return run_websocket_command(config_msg)

# Simple command functions
def halt():
    """Send HALT referee command - stops all robots immediately"""
    send_referee_command("HALT")

def stop():
    """Send STOP referee command - stops game but robots can move"""
    send_referee_command("STOP")

def force_start():
    """Send FORCE_START referee command - starts game immediately"""
    send_referee_command("FORCE_START")

def normal_start():
    """Send NORMAL_START referee command - starts game normally"""
    send_referee_command("NORMAL_START")

def direct_free_kick(team):
    """Send DIRECT referee command - awards direct free kick to team"""
    send_referee_command("DIRECT", team)

def kickoff(team):
    """Send KICKOFF referee command - starts kickoff for team"""
    send_referee_command("KICKOFF", team)

def penalty(team):
    """Send PENALTY referee command - starts penalty for team"""
    send_referee_command("PENALTY", team)

def ball_placement(team):
    """Send BALL_PLACEMENT referee command - starts ball placement for team"""
    send_referee_command("BALL_PLACEMENT", team)

def timeout(team):
    """Send TIMEOUT referee command - starts timeout for team"""
    send_referee_command("TIMEOUT", team)

def start_game():
    """Set Blue team as first kickoff, on positive half, and start the game properly"""
    # Set sides for teams
    set_team_state("BLUE", True)  # Blue on positive half
    set_team_state("YELLOW", False)  # Yellow on negative half
    
    # Set Blue for first kickoff
    set_first_kickoff_team("YELLOW")
    
    # Start sequence
    halt()  # First halt to ensure safe state
    # stop()  # Then stop to prepare for start
    kickoff("BLUE")  # Set up kickoff for Blue team

if __name__ == "__main__":
    print("Connecting to game controller...")
    
    start_game()
