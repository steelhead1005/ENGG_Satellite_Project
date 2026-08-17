import serial
import csv
import time

# Configure the serial port and baud rate to match your Arduino
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600 
INITIAL_TIME = time.time()

def main():
    # 1. Open the serial connection
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Successfully connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException as e:
        print(f"Error: Could not open port {SERIAL_PORT}. Is it plugged in?")
        return

    # 2. Open a new CSV file and set up the writer
    with open('sentinel_telemetry.csv', mode='w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        
        # Write the column headers
        csv_writer.writerow(['Timestamp', 'Emitter_State', 'IR_Status', 'Motor_State', 'Laser_State'])
        print("Logging started. Press Ctrl+C to stop.")

        try:
            # 3. Continuous listening loop
            while True:
                if ser.in_waiting > 0:
                    # Read the raw byte data and decode it into a string
                    line = ser.readline().decode('utf-8').strip()

                    # 4. Parse the specific telemetry line from your Arduino code
                    # Expected format: "Emitter: ON | IR: DETECTED | Motor: ON | Laser: ON"
                    if "Emitter:" in line and "|" in line:
                        try:
                            # Split the string by the pipe character
                            parts = [p.strip() for p in line.split('|')]
                            
                            # Extract just the values (e.g., separating "Emitter: ON" to just "ON")
                            emitter_val = parts[0].split(':')[1].strip()
                            ir_val = parts[1].split(':')[1].strip()
                            motor_val = parts[2].split(':')[1].strip()
                            laser_val = parts[3].split(':')[1].strip()
                            currentTime = (time.time() - INITIAL_TIME)
                            # Write the parsed values to the CSV row
                            csv_writer.writerow([round(currentTime, 3), emitter_val, ir_val, motor_val, laser_val])
                            
                        except IndexError:
                            print(f"Malformed data skipped: {line}")
                    else:
                        # Print the standard event triggers (e.g., "MOTOR ON", "LASER OFF") to the terminal
                        print(f"Event Triggered: {line}")

        except KeyboardInterrupt:
            # Gracefully handle the user stopping the script
            print("\nLogging terminated. File saved as sentinel_telemetry.csv.")
        finally:
            ser.close()

if __name__ == '__main__':
    main()
