import sys
import os
import websockets
import asyncio
import json
import time

async def set_team_state(team, on_positive_half):
    """
    Set team state including which half they play on.
    
    Args:
        team (str): Either "BLUE" or "YELLOW"
        on_positive_half (bool): True if team should play on positive half
    """
    uri = "ws://localhost:8081/api/control"
    
    try:
        async with websockets.connect(uri) as websocket:
            # Similar structure to first_kickoff_team
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
                print("Received response:", json.loads(response))
            except asyncio.TimeoutError:
                print("No response received in 2 seconds")
                
    except websockets.exceptions.ConnectionClosed as e:
        print(f"WebSocket connection closed: {e}")
    except Exception as e:
        print(f"Error: {e}")

async def send_referee_command(command_type, team=None):
    """
    Send a referee command using websockets.
    
    Args:
        command_type (str): One of: 
            HALT, STOP, NORMAL_START, FORCE_START, DIRECT, 
            KICKOFF, PENALTY, TIMEOUT, BALL_PLACEMENT
        team (str, optional): For team-specific commands, either "YELLOW" or "BLUE"
    """
    uri = "ws://localhost:8081/api/control"
    
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
                print("Received response:", json.loads(response))
            except asyncio.TimeoutError:
                print("No response received in 2 seconds")
                
    except websockets.exceptions.ConnectionClosed as e:
        print(f"WebSocket connection closed: {e}")
    except Exception as e:
        print(f"Error: {e}")

async def set_first_kickoff_team(team):
    """
    Set which team takes the first kickoff.
    
    Args:
        team (str): Either "BLUE" or "YELLOW"
    """
    uri = "ws://localhost:8081/api/control"
    
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
                print("Received response:", json.loads(response))
            except asyncio.TimeoutError:
                print("No response received in 2 seconds")
                
    except websockets.exceptions.ConnectionClosed as e:
        print(f"WebSocket connection closed: {e}")
    except Exception as e:
        print(f"Error: {e}")

async def halt():
    """Send HALT referee command - stops all robots immediately"""
    await send_referee_command("HALT")

async def stop():
    """Send STOP referee command - stops game but robots can move"""
    await send_referee_command("STOP")

async def force_start():
    """Send FORCE_START referee command - starts game immediately"""
    await send_referee_command("FORCE_START")

async def normal_start():
    """Send NORMAL_START referee command - starts game normally"""
    await send_referee_command("NORMAL_START")

async def direct_free_kick(team):
    """Send DIRECT referee command - awards direct free kick to team"""
    await send_referee_command("DIRECT", team)

async def kickoff(team):
    """Send KICKOFF referee command - starts kickoff for team"""
    await send_referee_command("KICKOFF", team)

async def penalty(team):
    """Send PENALTY referee command - starts penalty for team"""
    await send_referee_command("PENALTY", team)

async def ball_placement(team):
    """Send BALL_PLACEMENT referee command - starts ball placement for team"""
    await send_referee_command("BALL_PLACEMENT", team)

async def timeout(team):
    """Send TIMEOUT referee command - starts timeout for team"""
    await send_referee_command("TIMEOUT", team)

async def start_game():
    """Set Blue team as first kickoff, on positive half, and start the game properly"""
    # Set sides for teams
    await set_team_state("BLUE", True)  # Blue on positive half
    await set_team_state("YELLOW", False)  # Yellow on negative half
    
    # Set Blue for first kickoff
    await set_first_kickoff_team("BLUE")
    
    # Start sequence
    await halt()  # First halt to ensure safe state
    await stop()  # Then stop to prepare for start
    await asyncio.sleep(10)
    await kickoff("BLUE")  # Set up kickoff for Blue team
    # await normal_start()  # Start the game normally

if __name__ == "__main__":
    print("Connecting to game controller...")
    
    # To set up and start a game with Blue kickoff and on positive half:
    asyncio.get_event_loop().run_until_complete(start_game())
    
    # Or use individual commands as needed:
    # asyncio.get_event_loop().run_until_complete(set_team_state("BLUE", True))
    # asyncio.get_event_loop().run_until_complete(halt())
    # asyncio.get_event_loop().run_until_complete(normal_start())