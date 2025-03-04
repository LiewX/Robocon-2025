import asyncio
import websockets
import csv
import time

received_messages = []  # List to store received messages

async def handle_client(websocket):
    print("Client connected")
    try:
        async for message in websocket:
            print(f"Received message: {message}")
            received_messages.append(message)  # Store message
    except websockets.exceptions.ConnectionClosed as e:
        print(f"Client disconnected: {e}")

async def write_to_csv():
    await asyncio.sleep(20)  # Wait 15 seconds before writing
    if received_messages:
        with open("messages.csv", "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["PWM", "UL", "UR", "BL", "BR"])  # CSV headers
            for msg in received_messages:
                tokens = msg.split(";")  # Tokenize message
                if len(tokens) == 5:
                    writer.writerow(tokens)  # Write row if correctly formatted
        print("Messages written to messages.csv")

async def main():
    server = await websockets.serve(handle_client, "0.0.0.0", 81)  # Listen on port 81
    print("WebSocket Server started on port 81")
    asyncio.create_task(write_to_csv())  # Start CSV writing task
    await server.wait_closed()

if __name__ == "__main__":
    asyncio.run(main())
