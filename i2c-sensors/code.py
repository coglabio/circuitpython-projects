import time
import board
import busio
import adafruit_bme680
import adafruit_vl6180x
import cedargrove_nau7802
import usb_cdc
import microcontroller
import json

serial = usb_cdc.console

# Initialize I2C bus and sensors
i2c = busio.I2C(board.SCL, board.SDA)

try:
    # Initialize BME680 sensor
    bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c)
    # Initialize VL6180X sensor
    vl6180x = adafruit_vl6180x.VL6180X(i2c)
    # Initialize NAU7802 ADC
    nau7802 = cedargrove_nau7802.NAU7802(i2c)
    # Enable channel 1 for weight readings
    nau7802.channel = 1
    nau7802.gain = 128  # High gain for small load cells
    sensors_ok = True
except Exception as e:
    serial.write(f'Error initializing sensors: {str(e)}\r\n'.encode('utf-8'))
    sensors_ok = False

id = ""
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
        id = identifier
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

id = read_identifier()

while True:
    if serial.in_waiting > 0:
        command_line = read_until(b'\r').decode('utf-8')  # Read the full command line
        # Split command and potential arguments
        parts = command_line.split(' ', 1)
        command = parts[0].strip()
        args = parts[1].strip() if len(parts) > 1 else ''

        # print(f"Received command: {command}")

        if command == '?':
            # Read BME680 sensor
            temp = bme680.temperature
            humidity = bme680.relative_humidity
            pressure = bme680.pressure
            gas = bme680.gas

            # Read VL6180X sensor
            range_mm = vl6180x.range
            lux = vl6180x.read_lux(adafruit_vl6180x.ALS_GAIN_1)

            # Read NAU7802 ADC (average of 5 readings)
            weight_sum = 0
            samples = 5
            for _ in range(samples):
                if nau7802.available:
                    weight_sum += nau7802.read()
                    time.sleep(0.01)  # Short delay between readings
            weight_raw = weight_sum / samples

            response = {
                'id': id,
                'data': {
                    'sensor_0': {
                        'index': 0,
                        'type': 'Temperature',
                        'value': round(temp, 2),
                        'unit': 'C'
                    },
                    'sensor_1': {
                        'index': 1,
                        'type': 'Humidity',
                        'value': round(humidity, 2),
                        'unit': '%'
                    },
                    'sensor_2': {
                        'index': 2,
                        'type': 'Pressure',
                        'value': round(pressure, 2),
                        'unit': 'hPa'
                    },
                    'sensor_3': {
                        'index': 3,
                        'type': 'Gas',
                        'value': round(gas, 2),
                        'unit': 'Ω'
                    },
                    'sensor_4': {
                        'index': 4,
                        'type': 'Distance',
                        'value': range_mm,
                        'unit': 'mm'
                    },
                    'sensor_5': {
                        'index': 5,
                        'type': 'Light',
                        'value': round(lux, 2),
                        'unit': 'lux'
                    },
                    'sensor_6': {
                        'index': 6,
                        'type': 'ADC',
                        'value': round(weight_raw, 2),
                        'unit': 'raw'
                    }
                }
            }
            serial.write(f'{json.dumps(response)}\r\n'.encode('utf-8'))
        elif command == 'setid':
            if args:
                # print(args)
                success = write_identifier(args)
                id = args
                if success:
                    serial.write(f'ok\r\n'.encode('utf-8'))
                else:
                    serial.write(f'error\r\n'.encode('utf-8'))
            else:
                serial.write(f'error: no identifier provided\r\n'.encode('utf-8'))
        elif command == 'getid':
            identifier = id
            response = {'id': identifier if identifier else None}
            serial.write(f'{json.dumps(response)}\r\n'.encode('utf-8'))
        else:
            serial.write(f'error: invalid command\r\n'.encode('utf-8'))
    time.sleep(0.01)  # Small delay to prevent overwhelming the CPU
