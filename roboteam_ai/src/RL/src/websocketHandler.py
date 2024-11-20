import os
import websockets
import asyncio
import json

IS_IN_K8S = True # We run it locally.

def get_websocket_uri():
    """Get the appropriate URI based on the environment"""
    if IS_IN_K8S:
        host = "roboteam-ray-worker-svc"
        print("Running in Kubernetes, using service DNS")
    else:
        host = "localhost"
        print("Running locally")
    
    return f"ws://{host}:8081/api/control"

async def send_websocket_message(message, timeout=2.0):
    """Generic function to send websocket messages"""
    uri = get_websocket_uri()
    try:
        async with websockets.connect(uri) as websocket:
            await websocket.send(json.dumps(message))
            return await asyncio.wait_for(websocket.recv(), timeout=timeout)
    except Exception as e:
        print(f"Websocket error: {e}")
        raise

def run_websocket_command(message):
    """
    Synchronous wrapper to run websocket commands
    This is the main file 
    """
    try:
        loop = asyncio.get_event_loop()
    except RuntimeError:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
    
    try:
        return loop.run_until_complete(send_websocket_message(message))
    finally:
        loop.close()
        asyncio.set_event_loop(None)