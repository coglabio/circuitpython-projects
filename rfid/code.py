import time
import board
import busio
import digitalio
import usb_cdc
import microcontroller
import json
import os
import adafruit_pn532.i2c

serial = usb_cdc.console

# Initialize I2C bus and sensor
sda = board.SDA
scl = board.SCL
i2c = None

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

# Initialize PN532 RFID reader
pn532 = None
try:
    if i2c is None:
        i2c = busio.I2C(scl, sda)
    pn532 = adafruit_pn532.i2c.PN532_I2C(i2c)
    pn532.SAM_configuration()  # Configure the secure access module
    available_sensors['pn532'] = True
    serial.write('PN532 RFID reader detected\r\n'.encode('utf-8'))
except Exception as e:
    serial.write(f'PN532 RFID reader not available: {str(e)}\r\n'.encode('utf-8'))

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

def uid_to_string(uid):
    """Convert a UID byte array to a hex string."""
    return ''.join(['%02X' % i for i in uid])

id = read_identifier()
if id == '':
    id = generate_uuid4()
    write_identifier(id)

while True:
    # Check for incoming commands
    if serial.in_waiting > 0:
        command_line = read_until(b'\r').decode('utf-8')  # Read the full command line
        # Split command and potential arguments
        parts = command_line.split(' ', 1)
        command = parts[0].strip()
        args = parts[1].strip() if len(parts) > 1 else ''

        if command == '?':
            response = {
                'id': id,
                'data': {}
            }

            # Check for currently present RFID card
            if pn532 is not None:
                try:
                    # Scan for RFID card only when the '?' command is received
                    uid = pn532.read_passive_target(timeout=0.1)
                    if uid is not None:
                        uid_string = uid_to_string(uid)
                        # Output the UID to serial
                        serial.write(f'Card detected! UID: {uid_string}\r\n'.encode('utf-8'))
                        response['data']['sensor_0'] = {
                            'index': 0,
                            'type': 'RFID',
                            'value': uid_string
                        }
                    else:
                        response['data']['sensor_0'] = {
                            'index': 0,
                            'type': 'RFID',
                            'value': None
                        }
                except Exception as e:
                    serial.write(f'error reading PN532 - {str(e)}\r\n'.encode('utf-8'))

            # Send the response back over serial
            serial.write(f'{json.dumps(response)}\r\n'.encode('utf-8'))

        elif command == 'setid':
            if args:
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
        else:
            serial.write('error: invalid command\r\n'.encode('utf-8'))
    time.sleep(0.01)  # Small delay to prevent overwhelming the CPU
