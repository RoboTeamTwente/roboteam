import sys
import os
import websockets
import asyncio
import json

def reset_referee_state():
    """
    Synchronous function to reset the referee state
    """
    uri = "ws://localhost:8081/api/control"
    
    async def _async_reset():
        try:
            async with websockets.connect(uri) as websocket:
                # Create JSON message
                reset_msg = {
                    "reset_match": True
                }
                
                print("Sending JSON reset command...")
                await websocket.send(json.dumps(reset_msg))
                
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
    loop.run_until_complete(_async_reset())

if __name__ == "__main__":
    print("Connecting to game controller...")
    asyncio.get_event_loop().run_until_complete(reset_referee_state())