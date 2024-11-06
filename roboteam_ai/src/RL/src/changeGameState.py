import sys
import os
import websockets
import asyncio
import json
import time

def set_team_state(team, on_positive_half):
    """
    Set team state including which half they play on.
    
    Args:
        team (str): Either "BLUE" or "YELLOW"
        on_positive_half (bool): True if team should play on positive half
    """
    uri = "ws://localhost:8081/api/control"
    
    async def _async_set_team():
        try:
            async with websockets.connect(uri) as websocket:
                state_msg = {
                    "change": {
                        "update_team_state_change": {
                            "for_team": team,
                            "on_positive_half": on_positive_half
                        }
                    }
                }
                
                print(f"Setting {team} team to play on {'positive' if on_positive_half else 'negative'} half...")
                await websocket.send(json.dumps(state_msg))
                
                try:
                    response = await asyncio.wait_for(websocket.recv(), timeout=2.0)
                    #print("Received response:", json.loads(response))
                except asyncio.TimeoutError:
                    print("No response received in 2 seconds")
                    
        except websockets.exceptions.ConnectionClosed as e:
            print(f"WebSocket connection closed: {e}")
        except Exception as e:
            print(f"Error: {e}")
    
    # Run the async function synchronously
    loop = asyncio.get_event_loop()
    loop.run_until_complete(_async_set_team())

def send_referee_command(command_type, team=None):
    """
    Send a referee command using websockets.
    
    Args:
        command_type (str): One of: 
            HALT, STOP, NORMAL_START, FORCE_START, DIRECT, 
            KICKOFF, PENALTY, TIMEOUT, BALL_PLACEMENT
        team (str, optional): For team-specific commands, either "YELLOW" or "BLUE"
    """
    uri = "ws://localhost:8081/api/control"
    
    async def _async_send_command():
        try:
            async with websockets.connect(uri) as websocket:
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
                
                print(f"Sending referee command: {command_type} {'for ' + team if team else ''}...")
                await websocket.send(json.dumps(command_msg))
                
                try:
                    response = await asyncio.wait_for(websocket.recv(), timeout=2.0)
                    #print("Received response:", json.loads(response))
                except asyncio.TimeoutError:
                    print("No response received in 2 seconds")
                    
        except websockets.exceptions.ConnectionClosed as e:
            print(f"WebSocket connection closed: {e}")
        except Exception as e:
            print(f"Error: {e}")
    
    loop = asyncio.get_event_loop()
    loop.run_until_complete(_async_send_command())

def set_first_kickoff_team(team):
    """
    Set which team takes the first kickoff.
    
    Args:
        team (str): Either "BLUE" or "YELLOW"
    """
    uri = "ws://localhost:8081/api/control"
    
    async def _async_set_kickoff():
        try:
            async with websockets.connect(uri) as websocket:
                config_msg = {
                    "change": {
                        "update_config_change": {
                            "first_kickoff_team": team
                        }
                    }
                }
                
                print(f"Setting first kickoff team to {team}...")
                await websocket.send(json.dumps(config_msg))
                
                try:
                    response = await asyncio.wait_for(websocket.recv(), timeout=2.0)
                    #print("Received response:", json.loads(response))
                except asyncio.TimeoutError:
                    print("No response received in 2 seconds")
                    
        except websockets.exceptions.ConnectionClosed as e:
            print(f"WebSocket connection closed: {e}")
        except Exception as e:
            print(f"Error: {e}")
    
    loop = asyncio.get_event_loop()
    loop.run_until_complete(_async_set_kickoff())

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
    set_first_kickoff_team("BLUE")
    
    # Start sequence
    # halt()  # First halt to ensure safe state
    # stop()  # Then stop to prepare for start
    # time.sleep(10)  # Regular sleep instead of asyncio.sleep
    kickoff("BLUE")  # Set up kickoff for Blue team
    # normal_start()  # Start the game normally

if __name__ == "__main__":
    print("Connecting to game controller...")
    
    start_game()
