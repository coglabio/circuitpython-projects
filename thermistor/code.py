import time
import board
import pwmio
import usb_cdc
import microcontroller
import json
import os
import adafruit_thermistor

print("Lab Sync - Thermistor Sensor")
print("Version: 1.0")
print("Copyright (C) 2025 - Lab Sync Inc.")
print("All rights reserved.")

print("Use the following commands:")
print("  ? - Get sensor data")
print("  setid [ID] - Set device identifier")
print("  getid - Get device identifier")

# Initialize serial
serial = usb_cdc.console

# Set up thermistor on A0 (GP26)
# Using typical 10K thermistor with 10K series resistor (low-side configuration)
# Parameters: pin, series_resistor, nominal_resistance, nominal_temp_C, b_coefficient
thermistor = adafruit_thermistor.Thermistor(board.A0, 10000, 2500, 25, 3950, high_side=False)

# For identifier management
def generate_uuid4():
    """Generate a random UUID4 string."""
    b = bytearray(os.urandom(16))
    b[6] = (b[6] & 0x0F) | 0x40  # UUIDv4
    b[8] = (b[8] & 0x3F) | 0x80  # Variant

    uuid_str = '{:02x}{:02x}{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}'.format(
        *b
    )
    return uuid_str

def read_identifier():
    """Read the device identifier from NVM."""
    try:
        nvm_bytes = microcontroller.nvm[0:36]  # GUID is max 36 chars
        # Find the null terminator if it exists
        null_pos = 0
        while null_pos < len(nvm_bytes) and nvm_bytes[null_pos] != 0:
            null_pos += 1
        return bytes(nvm_bytes[0:null_pos]).decode('utf-8')
    except:
        return ''

def write_identifier(identifier):
    """Write the device identifier to NVM."""
    # Ensure the identifier is not too long
    if len(identifier) > 36:
        return False
    try:
        # Convert string to bytes and pad with zeros
        id_bytes = identifier.encode('utf-8')
        padded = id_bytes + bytes([0] * (36 - len(id_bytes)))
        microcontroller.nvm[0:36] = padded
        return True
    except:
        return False

# Initialize device identifier
device_id = read_identifier()
if device_id == '':
    device_id = generate_uuid4()
    write_identifier(device_id)

def get_temperature():
    """Read the thermistor and return temperature in Celsius."""
    return thermistor.temperature

def read_until(terminator=b'\r'):  # Change this to any character you want
    buffer = b""
    while True:
        if serial.in_waiting > 0:
            char = serial.read(1)  # Read one byte at a time
            if char == terminator:  # Stop at the defined termination character
                return buffer
            buffer += char  # Append character to the buffer
# Main loop
while True:
    if serial.in_waiting > 0:
        command = read_until(b'\r').decode('utf-8')
        # print("Command: " + command)
        if command == "?":
            # Create JSON response with device ID and temperature reading
            temperature = get_temperature()
            response = {
                'id': device_id,
                'data': {
                    'sensor_0': {
                        'index': 0,
                        'type': 'Temperature',
                        'value': round(temperature, 2),
                        'unit': 'C'
                    },
                }
            }
            # Send the response back over serial
            serial.write(f'{json.dumps(response)}\r\n'.encode('utf-8'))
        
        elif command.startswith("setid "):
            # Extract ID value after 'setid '
            new_id = command[6:].strip()
            if new_id:
                success = write_identifier(new_id)
                device_id = new_id
                if success:
                    print("OK - Device ID set")
                else:
                    print("ERROR - Failed to set device ID")
            else:
                print("ERROR - No identifier provided")
        
        elif command == "getid":
            response = {'id': device_id}
            serial.write(f'{json.dumps(response)}\r\n'.encode('utf-8'))
                
        else:
            print(f"ERROR - Unknown command: {command}")
    
    time.sleep(0.01)  # Small delay to prevent CPU hogging
