import time
import board
import pwmio
import usb_cdc
import microcontroller
import analogio
import json
import os

print("Lab Sync - H-Bridge Motor Controller")
print("Version: 1.0")
print("Copyright (C) 2025 - Lab Sync Inc.")
print("All rights reserved.")

print("Use the following commands:")
print("  CW - Set direction to clockwise")
print("  CCW - Set direction to counter-clockwise")
print("  ON - Turn motor ON")
print("  OFF - Turn motor OFF")
print("  S[0.0-100.0] - Set motor speed")

# Set up the PWM outputs for H-Bridge control
# IN1 pin (GP6) - Controls one direction
motor_pwm_in1 = pwmio.PWMOut(board.GP6, frequency=5000, duty_cycle=0)
# IN2 pin (GP7) - Controls the other direction
motor_pwm_in2 = pwmio.PWMOut(board.GP7, frequency=5000, duty_cycle=0)

# Initialize serial
serial = usb_cdc.console

# Set up analog input on A0 (GP26)
analog_in = analogio.AnalogIn(board.A0)

# Motor state tracking
motor_running = False
motor_speed = 0.0  # 0-100%
motor_direction = "CW"  # Default direction

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

def get_voltage():
    """Read the analog input and convert to voltage."""
    # The default reference voltage is 3.3V
    # ADC is 16-bit, so max value is 65535
    reading = analog_in.value
    voltage = (reading * 3.3) / 65535.0
    return voltage

def set_speed(speed_percent):
    """Convert speed percentage (0-100) to duty cycle (0-65535) and apply to the correct H-Bridge pin."""
    global motor_speed
    # Clamp speed between 0 and 100
    motor_speed = max(0.0, min(100.0, speed_percent))
    
    if motor_running:
        # Convert percentage to 16-bit duty cycle
        duty = int((motor_speed / 100.0) * 65535)
        
        # Apply to correct pin based on direction
        if motor_direction == "CW":
            motor_pwm_in1.duty_cycle = duty
            motor_pwm_in2.duty_cycle = 0
        else:  # CCW
            motor_pwm_in1.duty_cycle = 0
            motor_pwm_in2.duty_cycle = duty



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
        if command == "CW":
            motor_direction = "CW"
            if motor_running:
                duty = int((motor_speed / 100.0) * 65535)
                motor_pwm_in1.duty_cycle = duty
                motor_pwm_in2.duty_cycle = 0
            print("OK - Direction set to clockwise")
        
        elif command == "CCW":
            motor_direction = "CCW"
            if motor_running:
                duty = int((motor_speed / 100.0) * 65535)
                motor_pwm_in1.duty_cycle = 0
                motor_pwm_in2.duty_cycle = duty
            print("OK - Direction set to counter-clockwise")
        
        elif command == "ON":
            if not motor_running:
                motor_running = True
                duty = int((motor_speed / 100.0) * 65535)
                if motor_direction == "CW":
                    motor_pwm_in1.duty_cycle = duty
                    motor_pwm_in2.duty_cycle = 0
                else:  # CCW
                    motor_pwm_in1.duty_cycle = 0
                    motor_pwm_in2.duty_cycle = duty
                print(f"OK - Motor turned ON at {motor_speed}% speed")
        
        elif command == "OFF":
            if motor_running:
                motor_running = False
                motor_pwm_in1.duty_cycle = 0
                motor_pwm_in2.duty_cycle = 0
                print("OK - Motor turned OFF")
        
        elif command.startswith("S"):
            try:
                # Extract speed value after 'S'
                speed_value = float(command[1:])
                set_speed(speed_value)
                if motor_running:
                    duty = int((motor_speed / 100.0) * 65535)
                    if motor_direction == "CW":
                        motor_pwm_in1.duty_cycle = duty
                        motor_pwm_in2.duty_cycle = 0
                    else:  # CCW
                        motor_pwm_in1.duty_cycle = 0
                        motor_pwm_in2.duty_cycle = duty
                print(f"OK - Speed set to {motor_speed}%")
            except ValueError:
                print("ERROR - Invalid speed command format. Use S[0.0-100.0]")
        
        elif command == "?":
            # Create JSON response with device ID and analog reading
            voltage = get_voltage()
            response = {
                'id': device_id,
                'data': {
                    'sensor_0': {
                        'index': 0,
                        'type': 'Voltage',
                        'value': round(voltage, 3),
                        'unit': 'V'
                    },
                    'sensor_1': {
                        'index': 1,
                        'type': 'Motor',
                        'speed': motor_speed,
                        'direction': motor_direction,
                        'running': motor_running
                    }
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
