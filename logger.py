import serial
import csv
import time
import struct

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

    # 2. Open both the CSV file (text) and the Binary file (wb)
    with open('sentinel_telemetry.csv', mode='w', newline='') as csv_file, \
         open('sentinel_telemetry.bin', mode='wb') as bin_file:
        
        csv_writer = csv.writer(csv_file)
        
        # Write the column headers for CSV
        csv_writer.writerow(['Timestamp', 'Emitter_State', 'IR_Status', 'Motor_State', 'Laser_State'])
        print("Logging started. Press Ctrl+C to stop.")

        try:
            # 3. Continuous listening loop
            while True:
                if ser.in_waiting > 0:
                    # Read the raw byte data and decode it into a string
                    line = ser.readline().decode('utf-8').strip()

                    # 4. Parse the specific telemetry line from your Arduino code
                    if "Emitter:" in line and "|" in line:
                        try:
                            # Split the string by the pipe character
                            parts = [p.strip() for p in line.split('|')]
                            
                            # Extract just the values
                            emitter_val = parts[0].split(':')[1].strip()
                            ir_val = parts[1].split(':')[1].strip()
                            motor_val = parts[2].split(':')[1].strip()
                            laser_val = parts[3].split(':')[1].strip()
                            currentTime = (time.time() - INITIAL_TIME)
                            
                            # Write text to CSV
                            csv_writer.writerow([round(currentTime, 3), emitter_val, ir_val, motor_val, laser_val])
                            
                            # Map text states to integer binaries (1 or 0)
                            emitter_bin = 1 if emitter_val == "ON" else 0
                            ir_bin = 1 if ir_val == "DETECTED" else 0
                            motor_bin = 1 if motor_val == "ON" else 0
                            laser_bin = 1 if laser_val == "ON" else 0
                            
                            # Pack into binary format: Little-endian float, and 4 unsigned bytes
                            binary_data = struct.pack('<fBBBB', currentTime, emitter_bin, ir_bin, motor_bin, laser_bin)
                            bin_file.write(binary_data)
                            
                        except IndexError:
                            print(f"Malformed data skipped: {line}")
                    else:
                        print(f"Event Triggered: {line}")

        except KeyboardInterrupt:
            print("\nLogging terminated. Files saved as sentinel_telemetry.csv and sentinel_telemetry.bin.")
        finally:
            ser.close()

if __name__ == '__main__':
    main()
