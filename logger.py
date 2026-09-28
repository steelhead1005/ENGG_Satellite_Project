import serial
import csv
import time

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600
INITIAL_TIME = time.time()

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Successfully connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException:
        print(f"Error: Could not open port {SERIAL_PORT}. Is it plugged in?")
        return

    with open('sentinel_telemetry_v2.csv', mode='w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([
            'Timestamp', 'Emitter_State', 'IR_Status',
            'Sensor_Array', 'Target_Angle', 'Motor_State',
            'Laser_State', 'Loop_Us'
        ])

        print("Logging started. Press Ctrl+C to stop.")
        try:
            while True:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if "Emitter:" in line and "|" in line:
                        try:
                            parts = [p.strip() for p in line.split('|')]
                            emitter_val = parts[0].split(':')[1].strip()
                            ir_val      = parts[1].split(':')[1].strip()
                            array_val   = parts[2].split(':')[1].strip()
                            angle_val   = parts[3].split(':')[1].strip()
                            motor_val   = parts[4].split(':')[1].strip()
                            laser_val   = parts[5].split(':')[1].strip()
                            loop_us_val = parts[6].split(':')[1].strip()

                            current_time = round(time.time() - INITIAL_TIME, 3)
                            csv_writer.writerow([
                                current_time, emitter_val, ir_val,
                                array_val, angle_val, motor_val,
                                laser_val, loop_us_val
                            ])
                            print(line)
                        except IndexError:
                            print(f"Malformed data skipped: {line}")
                    else:
                        print(f"Event Triggered: {line}")
        except KeyboardInterrupt:
            print("\nLogging terminated. File saved as sentinel_telemetry_v2.csv.")
        finally:
            ser.close()

if __name__ == '__main__':
    main()
