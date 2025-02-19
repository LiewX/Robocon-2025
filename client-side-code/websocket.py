from websocket import create_connection
import time
import csv

# Replace with your ESP32's IP and port
esp32_ip = "192.168.43.184:81"

# Set the timeout duration in seconds
timeout_duration = 15

ws = None  # Initialize WebSocket variable

with open('speedmap_data.csv', mode='a', newline='') as file:
    writer = csv.writer(file)

while True:
    try:
        print("Attempting to connect...")
        ws = create_connection(f"ws://{esp32_ip}")
        print("Connected to WebSocket server!")
        
        # Track the time of the last received message
        last_received_time = time.time()

        # Read data continuously from the WebSocket
        while True:
            try:
                # Set a timeout for receiving data
                ws.settimeout(timeout_duration)
                message = ws.recv()  # Receive data from ESP32
                print("Message from ESP32:", message)
                
                tokens = message.split(";")
                writer.writerow(tokens)
                print(f"Data written to CSV: {tokens}")

                # Update the last received time
                last_received_time = time.time()
            except Exception as e:
                # Check if the timeout has elapsed
                if time.time() - last_received_time > timeout_duration:
                    print(f"No data received for {timeout_duration} seconds. Disconnecting...")
                    break
                else:
                    print(f"Error while receiving data: {e}")
                    continue
    except Exception as e:
        print(f"Connection failed: {e}. Retrying in 3 seconds...")
        time.sleep(3)
    finally:
        # Close the WebSocket connection if it's open
        if ws is not None:
            ws.close()
            print("Connection closed. Attempting to reconnect...")
