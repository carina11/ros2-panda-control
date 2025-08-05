#!/usr/bin/python3
import asyncio
import websockets
import json

connected_clients = set()

async def handle_client(websocket):
    connected_clients.add(websocket)
    print("Client connected")

    try:
        async for message in websocket:
            print(f"Received: {message}")

            # Parse JSON
            try:
                data = json.loads(message)
                print("Parsed JSON:", data)
                response = {
                    "status": "received",
                    "original": data,
                }
            except json.JSONDecodeError:
                response = {
                    "status": "error",
                    "message": "Invalid JSON"
                }

            # Send JSON response
            await websocket.send(json.dumps(response))

    except websockets.ConnectionClosed:
        print("Client disconnected")
    finally:
        connected_clients.remove(websocket)

async def main():
    async with websockets.serve(handle_client, "0.0.0.0", 8080):
        print("WebSocket server started on ws://localhost:8080")
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())