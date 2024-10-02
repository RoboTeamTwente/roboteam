import asyncio
import websockets
from google.protobuf.json_format import MessageToJson, Parse
from ssl_gc_api_pb2 import Input, Output

async def reset_match(uri='ws://localhost:8081/api/control'):
    async with websockets.connect(uri) as websocket:
        # Create an Input message
        input_message = Input()
        input_message.reset_match = True

        # Convert the message to JSON
        json_message = MessageToJson(input_message)

        # Send the message
        await websocket.send(json_message)
        print(f"Sent reset command: {json_message}")

        # Wait for a response
        response = await websocket.recv()
        output_message = Parse(response, Output())
        print(f"Received response: {MessageToJson(output_message)}")

async def main():
    await reset_match()
    print("Reset command sent to SSL Game Controller")

if __name__ == "__main__":
    asyncio.run(main())