import time
import board
import pwmio
import usb_cdc
import microcontroller
import analogio
import digitalio
import json
import os

print("Lab Sync - Dual Motor Controller")
print("Version: 1.0")
print("Copyright (C) 2025 - Lab Sync Inc.")
print("All rights reserved.")

print("Use the following commands:")
print("  M1CW / M2CW - Set motor 1/2 direction to clockwise")
print("  M1CCW / M2CCW - Set motor 1/2 direction to counter-clockwise")
print("  M1ON / M2ON - Turn motor 1/2 ON")
print("  M1OFF / M2OFF - Turn motor 1/2 OFF")
print("  M1S[0.0-100.0] / M2S[0.0-100.0] - Set motor 1/2 speed")
print("  PULLUP - Set digital inputs to pull-up")
print("  PULLDOWN - Set digital inputs to pull-down")
print("  FLOAT - Set digital inputs to floating (no pull resistor)")
print("M1 pins: IN1=GP6, IN2=GP7")
print("M2 pins: IN1=GP4, IN2=GP5")
print("Analog inputs: A0-A3 (GP26-GP29)")
print("Digital inputs: GP0-GP3")
# Set up the PWM outputs for H-Bridge control
# Motor 1 - GP6 and GP7
motor1_pwm_in1 = pwmio.PWMOut(board.GP6, frequency=5000, duty_cycle=0)
motor1_pwm_in2 = pwmio.PWMOut(board.GP7, frequency=5000, duty_cycle=0)

# Motor 2 - GP4 and GP5
motor2_pwm_in1 = pwmio.PWMOut(board.GP4, frequency=5000, duty_cycle=0)
motor2_pwm_in2 = pwmio.PWMOut(board.GP5, frequency=5000, duty_cycle=0)

# Initialize serial
serial = usb_cdc.console

# Set up analog inputs on A0-A3 (GP26-GP29)
analog_inputs = [
    analogio.AnalogIn(board.A0),  # GP26
    analogio.AnalogIn(board.A1),  # GP27
    analogio.AnalogIn(board.A2),  # GP28
    analogio.AnalogIn(board.A3)   # GP29
]

# Digital input configuration will be set after reading from NVM
digital_inputs = []
digital_input_pins = [board.GP0, board.GP1, board.GP2, board.GP3]

# Motor 1 state tracking
motor1_running = False
motor1_speed = 0.0  # 0-100%
motor1_direction = "CW"  # Default direction

# Motor 2 state tracking
motor2_running = False
motor2_speed = 0.0  # 0-100%
motor2_direction = "CW"  # Default direction

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

def read_pull_config():
    """Read the pull resistor configuration from NVM."""
    # NVM byte 36 stores pull config: 0 = pull-up (default), 1 = pull-down, 2 = float
    try:
        config = microcontroller.nvm[36]
        if config == 1:
            return 'DOWN'
        elif config == 2:
            return 'FLOAT'
        else:
            return 'UP'
    except:
        return 'UP'

def write_pull_config(pull_type):
    """Write the pull resistor configuration to NVM."""
    try:
        if pull_type == 'DOWN':
            microcontroller.nvm[36] = 1
        elif pull_type == 'FLOAT':
            microcontroller.nvm[36] = 2
        else:  # 'UP'
            microcontroller.nvm[36] = 0
        return True
    except:
        return False

def configure_digital_inputs(pull_type):
    """Configure all digital inputs with specified pull resistor."""
    global digital_inputs
    # Deinitialize existing pins
    for pin in digital_inputs:
        pin.deinit()
    digital_inputs.clear()
    
    # Reconfigure with new pull setting
    if pull_type == 'DOWN':
        pull_mode = digitalio.Pull.DOWN
    elif pull_type == 'FLOAT':
        pull_mode = None  # No pull resistor
    else:  # 'UP'
        pull_mode = digitalio.Pull.UP
    
    for pin in digital_input_pins:
        digital_pin = digitalio.DigitalInOut(pin)
        digital_pin.direction = digitalio.Direction.INPUT
        digital_pin.pull = pull_mode
        digital_inputs.append(digital_pin)
    
    return pull_type

# Initialize device identifier
device_id = read_identifier()
if device_id == '':
    device_id = generate_uuid4()
    write_identifier(device_id)

# Initialize digital inputs with stored pull configuration
pull_config = read_pull_config()
configure_digital_inputs(pull_config)
print(f"Digital inputs configured with pull-{pull_config.lower()}")

def get_voltage(channel):
    """Read the analog input and convert to voltage."""
    # The default reference voltage is 3.3V
    # ADC is 16-bit, so max value is 65535
    if 0 <= channel < len(analog_inputs):
        reading = analog_inputs[channel].value
        voltage = (reading * 3.3) / 65535.0
        return voltage
    return 0.0

def set_motor_speed(motor_num, speed_percent):
    """Convert speed percentage (0-100) to duty cycle (0-65535) and apply to the correct H-Bridge pin."""
    if motor_num == 1:
        global motor1_speed
        motor1_speed = max(0.0, min(100.0, speed_percent))
        
        if motor1_running:
            duty = int((motor1_speed / 100.0) * 65535)
            if motor1_direction == "CW":
                motor1_pwm_in1.duty_cycle = duty
                motor1_pwm_in2.duty_cycle = 0
            else:  # CCW
                motor1_pwm_in1.duty_cycle = 0
                motor1_pwm_in2.duty_cycle = duty
    
    elif motor_num == 2:
        global motor2_speed
        motor2_speed = max(0.0, min(100.0, speed_percent))
        
        if motor2_running:
            duty = int((motor2_speed / 100.0) * 65535)
            if motor2_direction == "CW":
                motor2_pwm_in1.duty_cycle = duty
                motor2_pwm_in2.duty_cycle = 0
            else:  # CCW
                motor2_pwm_in1.duty_cycle = 0
                motor2_pwm_in2.duty_cycle = duty



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
        
        # Motor 1 Commands
        if command == "M1CW":
            motor1_direction = "CW"
            if motor1_running:
                duty = int((motor1_speed / 100.0) * 65535)
                motor1_pwm_in1.duty_cycle = duty
                motor1_pwm_in2.duty_cycle = 0
            print("OK - Motor 1 direction set to clockwise")
        
        elif command == "M1CCW":
            motor1_direction = "CCW"
            if motor1_running:
                duty = int((motor1_speed / 100.0) * 65535)
                motor1_pwm_in1.duty_cycle = 0
                motor1_pwm_in2.duty_cycle = duty
            print("OK - Motor 1 direction set to counter-clockwise")
        
        elif command == "M1ON":
            if not motor1_running:
                motor1_running = True
                duty = int((motor1_speed / 100.0) * 65535)
                if motor1_direction == "CW":
                    motor1_pwm_in1.duty_cycle = duty
                    motor1_pwm_in2.duty_cycle = 0
                else:  # CCW
                    motor1_pwm_in1.duty_cycle = 0
                    motor1_pwm_in2.duty_cycle = duty
                print(f"OK - Motor 1 turned ON at {motor1_speed}% speed")
        
        elif command == "M1OFF":
            if motor1_running:
                motor1_running = False
                motor1_pwm_in1.duty_cycle = 0
                motor1_pwm_in2.duty_cycle = 0
                print("OK - Motor 1 turned OFF")
        
        elif command.startswith("M1S"):
            try:
                speed_value = float(command[3:])
                set_motor_speed(1, speed_value)
                if motor1_running:
                    duty = int((motor1_speed / 100.0) * 65535)
                    if motor1_direction == "CW":
                        motor1_pwm_in1.duty_cycle = duty
                        motor1_pwm_in2.duty_cycle = 0
                    else:  # CCW
                        motor1_pwm_in1.duty_cycle = 0
                        motor1_pwm_in2.duty_cycle = duty
                print(f"OK - Motor 1 speed set to {motor1_speed}%")
            except ValueError:
                print("ERROR - Invalid speed command format. Use M1S[0.0-100.0]")
        
        # Motor 2 Commands
        elif command == "M2CW":
            motor2_direction = "CW"
            if motor2_running:
                duty = int((motor2_speed / 100.0) * 65535)
                motor2_pwm_in1.duty_cycle = duty
                motor2_pwm_in2.duty_cycle = 0
            print("OK - Motor 2 direction set to clockwise")
        
        elif command == "M2CCW":
            motor2_direction = "CCW"
            if motor2_running:
                duty = int((motor2_speed / 100.0) * 65535)
                motor2_pwm_in1.duty_cycle = 0
                motor2_pwm_in2.duty_cycle = duty
            print("OK - Motor 2 direction set to counter-clockwise")
        
        elif command == "M2ON":
            if not motor2_running:
                motor2_running = True
                duty = int((motor2_speed / 100.0) * 65535)
                if motor2_direction == "CW":
                    motor2_pwm_in1.duty_cycle = duty
                    motor2_pwm_in2.duty_cycle = 0
                else:  # CCW
                    motor2_pwm_in1.duty_cycle = 0
                    motor2_pwm_in2.duty_cycle = duty
                print(f"OK - Motor 2 turned ON at {motor2_speed}% speed")
        
        elif command == "M2OFF":
            if motor2_running:
                motor2_running = False
                motor2_pwm_in1.duty_cycle = 0
                motor2_pwm_in2.duty_cycle = 0
                print("OK - Motor 2 turned OFF")
        
        elif command.startswith("M2S"):
            try:
                speed_value = float(command[3:])
                set_motor_speed(2, speed_value)
                if motor2_running:
                    duty = int((motor2_speed / 100.0) * 65535)
                    if motor2_direction == "CW":
                        motor2_pwm_in1.duty_cycle = duty
                        motor2_pwm_in2.duty_cycle = 0
                    else:  # CCW
                        motor2_pwm_in1.duty_cycle = 0
                        motor2_pwm_in2.duty_cycle = duty
                print(f"OK - Motor 2 speed set to {motor2_speed}%")
            except ValueError:
                print("ERROR - Invalid speed command format. Use M2S[0.0-100.0]")
        
        elif command == "?":
            # Create JSON response with device ID and all sensor readings
            response = {
                'id': device_id,
                'data': {}
            }
            
            # Add analog inputs (A0-A3)
            for i in range(len(analog_inputs)):
                response['data'][f'analog_{i}'] = {
                    'index': i,
                    'type': 'Voltage',
                    'channel': i,
                    'value': round(get_voltage(i), 3),
                    'unit': 'V',
                    'pin': f'A{i}'
                }
            
            # Add digital inputs (GP0-GP3)
            for i in range(len(digital_inputs)):
                response['data'][f'digital_{i}'] = {
                    'index': 4 + i,
                    'type': 'Digital',
                    'channel': i,
                    'value': digital_inputs[i].value,
                    'pin': f'GP{i}'
                }
            
            # Add motor 1 status
            response['data']['motor_1'] = {
                'index': 8,
                'type': 'Motor',
                'motor': 1,
                'speed': motor1_speed,
                'direction': motor1_direction,
                'running': motor1_running
            }
            
            # Add motor 2 status
            response['data']['motor_2'] = {
                'index': 9,
                'type': 'Motor',
                'motor': 2,
                'speed': motor2_speed,
                'direction': motor2_direction,
                'running': motor2_running
            }
            
            # Add pull configuration
            response['data']['config'] = {
                'pull_resistor': pull_config
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
        
        elif command == "PULLUP":
            pull_config = configure_digital_inputs('UP')
            write_pull_config('UP')
            print("OK - Digital inputs set to pull-up")
        
        elif command == "PULLDOWN":
            pull_config = configure_digital_inputs('DOWN')
            write_pull_config('DOWN')
            print("OK - Digital inputs set to pull-down")
        
        elif command == "FLOAT":
            pull_config = configure_digital_inputs('FLOAT')
            write_pull_config('FLOAT')
            print("OK - Digital inputs set to floating (no pull resistor)")
                
        else:
            print(f"ERROR - Unknown command: {command}")
    
    time.sleep(0.01)  # Small delay to prevent CPU hogging
