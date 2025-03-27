import time
import board
import busio
import digitalio
import cedargrove_nau7802
import usb_cdc
import microcontroller
import json
import os

serial = usb_cdc.console

# Initialize I2C bus and sensors
sda = board.SDA
scl = board.SCL
i2c = None

# Initialize min/max tracking variables - declaring them globally
nau_min_value = None
nau_max_value = None

def generate_uuid4():
    b = bytearray(os.urandom(16))
    b[6] = (b[6] & 0x0F) | 0x40  # UUIDv4
    b[8] = (b[8] & 0x3F) | 0x80  # Variant

    uuid_str = '{:02x}{:02x}{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}'.format(
        *b
    )
    return uuid_str

# Initialize variables
available_sensors = {}

# Initialize NAU7802 ADC (weight)
nau7802 = None
try:
    if i2c is None:
        i2c = busio.I2C(scl, sda)
    nau7802 = cedargrove_nau7802.NAU7802(i2c)
    # Enable channel 1 for weight readings
    nau7802.channel = 1
    nau7802.gain = 128  # High gain for small load cells
    available_sensors['nau7802'] = True
    serial.write('NAU7802 sensor detected\r\n'.encode('utf-8'))
    
    # Initialize min/max with first reading if sensor is available
    if nau7802.available:
        initial_value = nau7802.read()
        # No need for global declaration here as we're using module level variables
        nau_min_value = initial_value
        nau_max_value = initial_value
except Exception as e:
    serial.write(f'NAU7802 sensor not available: {str(e)}\r\n'.encode('utf-8'))

def read_identifier():
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

def read_until(terminator=b'\r'):  # Change this to any character you want
    buffer = b""
    while True:
        if serial.in_waiting > 0:
            char = serial.read(1)  # Read one byte at a time
            if char == terminator:  # Stop at the defined termination character
                return buffer
            buffer += char  # Append character to the buffer

def convert_signed_24bit(value):
    """Convert a 24-bit unsigned value to signed."""
    value &= 0xFFFFFF  # Mask to 24 bits
    if value & 0x800000:  # If the sign bit (MSB in 24-bit) is set
        value -= 0x1000000  # Convert to negative using two's complement
    return value

id = read_identifier()
if id == '':
    id = generate_uuid4()
    write_identifier(id)

# For timing continuous sampling
last_sample_time = 0
sample_interval = 0.1  # Sample every 100ms

while True:
    # Continuous sampling for min/max tracking
    current_time = time.monotonic()
    if (nau7802 is not None and 
            current_time - last_sample_time >= sample_interval):
        if nau7802.available:
            try:
                current_reading = nau7802.read()
                # Update min/max values
                if nau_min_value is None or current_reading < nau_min_value:
                    nau_min_value = current_reading
                if nau_max_value is None or current_reading > nau_max_value:
                    nau_max_value = current_reading
            except Exception:
                # Silently ignore errors during background sampling
                pass
        last_sample_time = current_time

    # Check for incoming commands
    if serial.in_waiting > 0:
        command_line = read_until(b'\r').decode('utf-8')  # Read the full command line
        # Split command and potential arguments
        parts = command_line.split(' ', 1)
        command = parts[0].strip()
        args = parts[1].strip() if len(parts) > 1 else ''

        # print(f"Received command: {command}")

        if command == '?':
            response = {
                'id': id,
                'data': {}
            }

            sensor_index = 0

            # Only read NAU7802 ADC if available
            if nau7802 is not None:
                try:
                    # Average of 5 readings
                    weight_sum = 0
                    samples = 5
                    samples_read = 0
                    for _ in range(samples):
                        if nau7802.available:
                            current_reading = nau7802.read()
                            weight_sum += current_reading
                            samples_read += 1
                            time.sleep(0.01)  # Short delay between readings

                    if samples_read > 0:
                        weight_raw = weight_sum / samples_read
                        # Add ADC data with min/max values
                        response['data'][f'sensor_{sensor_index}'] = {
                            'index': sensor_index,
                            'type': 'ADC',
                            'value': round(weight_raw, 2),
                            'min': (round(nau_min_value, 2) 
                                   if nau_min_value is not None else None),
                            'max': (round(nau_max_value, 2) 
                                   if nau_max_value is not None else None),
                            'unit': 'raw'
                        }
                        sensor_index += 1
                except Exception as e:
                    serial.write(f'error reading NAU7802 - {str(e)}\r\n'.encode('utf-8'))

            # Send the response back over serial
            serial.write(f'{json.dumps(response)}\r\n'.encode('utf-8'))

        elif command == 'setid':
            if args:
                # print(args)
                success = write_identifier(args)
                id = args
                if success:
                    serial.write('ok\r\n'.encode('utf-8'))
                else:
                    serial.write('error\r\n'.encode('utf-8'))
            else:
                serial.write('error: no identifier provided\r\n'.encode('utf-8'))
        elif command == 'getid':
            identifier = id
            response = {'id': identifier if identifier else None}
            serial.write(f'{json.dumps(response)}\r\n'.encode('utf-8'))
        elif command == 'resetminmax':
            # Reset min/max values to current reading if available
            if nau7802 is not None and nau7802.available:
                current_value = nau7802.read()
                nau_min_value = current_value
                nau_max_value = current_value
                serial.write('ok - min/max reset\r\n'.encode('utf-8'))
            else:
                serial.write('error: sensor not available\r\n'.encode('utf-8'))
        else:
            serial.write('error: invalid command\r\n'.encode('utf-8'))
    time.sleep(0.01)  # Small delay to prevent overwhelming the CPU
