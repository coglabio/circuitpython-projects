#!/usr/bin/env python3
"""
Temperature Logger for CircuitPython Thermistor Sensor
Connects to COM1 and logs temperature data to CSV file with timestamps
"""

import serial
import csv
import time
from datetime import datetime
import sys
import os

# Configuration
SERIAL_PORT = 'COM1'
BAUD_RATE = 9600
CSV_FILENAME = 'temperature_log.csv'
QUERY_INTERVAL = 1.0  # seconds

def setup_csv_file(filename):
    """Create CSV file with headers if it doesn't exist"""
    file_exists = os.path.exists(filename)
    
    if not file_exists:
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Timestamp', 'Temperature_C', 'Device_ID'])
        print(f"Created new CSV file: {filename}")
    else:
        print(f"Appending to existing CSV file: {filename}")

def connect_to_sensor():
    """Connect to the CircuitPython sensor on COM1"""
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=2.0,
            write_timeout=2.0
        )
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
        
        # Wait a moment for connection to stabilize
        time.sleep(1)
        
        # Clear any existing data in buffer
        ser.reset_input_buffer()
        
        return ser
    except serial.SerialException as e:
        print(f"Error connecting to {SERIAL_PORT}: {e}")
        print("Make sure the device is connected and COM1 is available.")
        return None

def query_temperature(ser):
    """Send query command and parse response"""
    try:
        # Send query command
        ser.write(b'?\r')
        ser.flush()
        
        # Read response with timeout
        response = ser.readline().decode('utf-8').strip()
        
        if response:
            try:
                # Parse JSON response
                import json
                data = json.loads(response)
                
                # Extract temperature and device ID from actual structure
                # Structure: {'id': device_id, 'data': {'sensor_0': {'value': temp, 'unit': 'C'}}}
                device_id = data.get('id', 'unknown')
                sensor_data = data.get('data', {}).get('sensor_0', {})
                temp_c = sensor_data.get('value')
                
                return temp_c, device_id
            except json.JSONDecodeError:
                print(f"Invalid JSON response: {response}")
                return None, None
        else:
            print("No response from sensor")
            return None, None
            
    except Exception as e:
        print(f"Error querying temperature: {e}")
        return None, None

def log_temperature(filename, timestamp, temp_c, device_id):
    """Log temperature data to CSV file"""
    try:
        with open(filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([timestamp, temp_c, device_id])
        return True
    except Exception as e:
        print(f"Error writing to CSV: {e}")
        return False

def main():
    """Main logging loop"""
    print("CircuitPython Thermistor Temperature Logger")
    print("=" * 50)
    print(f"Port: {SERIAL_PORT}")
    print(f"Baud Rate: {BAUD_RATE}")
    print(f"CSV File: {CSV_FILENAME}")
    print(f"Query Interval: {QUERY_INTERVAL} seconds")
    print("Press Ctrl+C to stop logging")
    print("=" * 50)
    
    # Setup CSV file
    setup_csv_file(CSV_FILENAME)
    
    # Connect to sensor
    ser = connect_to_sensor()
    if not ser:
        sys.exit(1)
    
    try:
        while True:
            # Get current timestamp
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            
            # Query temperature
            temp_c, device_id = query_temperature(ser)
            
            if temp_c is not None:
                # Log to CSV
                if log_temperature(CSV_FILENAME, timestamp, temp_c, device_id):
                    print(f"{timestamp} | {temp_c:.2f}°C | {device_id}")
                else:
                    print(f"{timestamp} | ERROR: Failed to log data")
            else:
                print(f"{timestamp} | ERROR: Failed to read temperature")
            
            # Wait before next query
            time.sleep(QUERY_INTERVAL)
            
    except KeyboardInterrupt:
        print("\nLogging stopped by user")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial connection closed")

if __name__ == "__main__":
    main()
