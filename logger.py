import serial
import csv
import time

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600
OUTPUT_FILE = 'run1_v1_baseline.csv'

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        print(f"Connected to {SERIAL_PORT}. Waiting 2s for Arduino reboot...")
        time.sleep(2.0)
        ser.write(b'D')
        print("Sent 'D' command. Downloading EEPROM telemetry...")
    except serial.SerialException:
        print(f"Error: Could not open {SERIAL_PORT}.")
        return

    sample_index = 0
    with open(OUTPUT_FILE, mode='w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([
            'Timestamp', 'Emitter_State', 'IR_Status',
            'Sensor_Array', 'Target_Angle', 'Motor_State',
            'Laser_State', 'Loop_Us'
        ])

        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            if "DUMP_COMPLETE" in line:
                print(f"\nDownload finished! {sample_index} frames saved to {OUTPUT_FILE}")
                break
            if "Emitter:" in line and "|" in line:
                parts = [p.strip() for p in line.split('|')]
                timestamp = round(sample_index * 0.25, 2)
                csv_writer.writerow([
                    timestamp,
                    parts[0].split(':')[1].strip(),
                    parts[1].split(':')[1].strip(),
                    parts[2].split(':')[1].strip(),
                    parts[3].split(':')[1].strip(),
                    parts[4].split(':')[1].strip(),
                    parts[5].split(':')[1].strip(),
                    parts[6].split(':')[1].strip()
                ])
                print(f"[{timestamp}s] {line}")
                sample_index += 1

    ser.close()

if __name__ == '__main__':
    main()
