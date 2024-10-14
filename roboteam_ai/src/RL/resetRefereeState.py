"""
resetRefereeState is a script to reset the referee state of the game (it basically resets the match (clock))
"""

import asyncio
import websockets
from google.protobuf.json_format import MessageToJson, Parse
import os
import sys

# Make sure to go back to the main roboteam directory
current_dir = os.path.dirname(os.path.abspath(__file__))
roboteam_path = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))

# Add to sys.path
sys.path.append(roboteam_path)

# Now import the generated protobuf classes
from roboteam_networking.proto.ssl_gc_api_pb2 import Input
from roboteam_networking.proto.ssl_gc_change_pb2 import Change
from roboteam_networking.proto.ssl_gc_common_pb2 import Team
from roboteam_networking.proto.ssl_gc_state_pb2 import Command


async def reset_and_stop_match(uri='ws://localhost:8081/api/control'):
    async with websockets.connect(uri) as websocket:
        
        # Step 1: Reset the match
        reset_message = Input(reset_match=True)
        await websocket.send(MessageToJson(reset_message))
        response = await websocket.recv()

        # Step 2: Send STOP command
        stop_message = Input(
            change=Change(
                new_command_change=Change.NewCommand(
                    command=Command(
                        type=Command.Type.STOP,
                        for_team=Team.UNKNOWN
                    )
                )
            )
        )
        await websocket.send(MessageToJson(stop_message))
        print(f"Sent STOP command: {MessageToJson(stop_message)}")
        response = await websocket.recv()

        print("Reset and STOP commands sent to SSL Game Controller")

if __name__ == "__main__":
    asyncio.run(reset_and_stop_match())