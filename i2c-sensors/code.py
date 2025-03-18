import time
import board
import busio
import digitalio
import adafruit_bme680
import adafruit_vl6180x
import cedargrove_nau7802
import usb_cdc
import microcontroller
import json
from adafruit_hx711.hx711 import HX711
from adafruit_hx711.analog_in import AnalogIn

serial = usb_cdc.console

# Initialize I2C bus and sensors
# i2c = busio.I2C(board.SCL, board.SDA)

data = digitalio.DigitalInOut(board.SDA)
data.direction = digitalio.Direction.INPUT
clock = digitalio.DigitalInOut(board.SCL)
clock.direction = digitalio.Direction.OUTPUT

# Initialize sensors individually and track which ones are available
available_sensors = {
    'bme680': False,
    'vl6180x': False,
    'nau7802': False,
    'hx711': False
}

# Initialize BME680 sensor (temperature, humidity, pressure, gas)
bme680 = None
try:
    bme680 = adafruit_bme680.Adafruit_BME680_I2C(data,clock)
    available_sensors['bme680'] = True
    serial.write(f'BME680 sensor detected\r\n'.encode('utf-8'))
except Exception as e:
    serial.write(f'BME680 sensor not available: {str(e)}\r\n'.encode('utf-8'))

# Initialize VL6180X sensor (distance, light)
vl6180x = None
try:
    vl6180x = adafruit_vl6180x.VL6180X(data, clock)
    available_sensors['vl6180x'] = True
    serial.write(f'VL6180X sensor detected\r\n'.encode('utf-8'))
except Exception as e:
    serial.write(f'VL6180X sensor not available: {str(e)}\r\n'.encode('utf-8'))

# Initialize NAU7802 ADC (weight)
nau7802 = None
try:
    nau7802 = cedargrove_nau7802.NAU7802(data, clock)
    # Enable channel 1 for weight readings
    nau7802.channel = 1
    nau7802.gain = 128  # High gain for small load cells
    available_sensors['nau7802'] = True
    serial.write(f'NAU7802 sensor detected\r\n'.encode('utf-8'))
except Exception as e:
    serial.write(f'NAU7802 sensor not available: {str(e)}\r\n'.encode('utf-8'))

# Initialize HX711 load cell ADC
hx711 = None
channel_a = None
try:
    # Set up HX711 pins - using direct digital pins, not I2C

    
    # Initialize HX711 with direct pin connections
    hx711 = HX711(data, clock)
    channel_a = AnalogIn(hx711, HX711.CHAN_A_GAIN_128)
    available_sensors['hx711'] = True
    serial.write(f'HX711 sensor detected\r\n'.encode('utf-8'))
except Exception as e:
    serial.write(f'HX711 sensor not available: {str(e)}\r\n'.encode('utf-8'))

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

def convert_signed_24bit(value):
    """Convert a 24-bit unsigned value to signed."""
    value &= 0xFFFFFF  # Mask to 24 bits
    if value & 0x800000:  # If the sign bit (MSB in 24-bit) is set
        value -= 0x1000000  # Convert to negative using two's complement
    return value

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
            response = {
                'id': id,
                'data': {}
            }
            
            sensor_index = 0
            
            # Only read BME680 sensor if available
            if available_sensors['bme680'] and bme680 is not None:
                try:
                    temp = bme680.temperature
                    humidity = bme680.relative_humidity
                    pressure = bme680.pressure
                    gas = bme680.gas
                    
                    # Add temperature data
                    response['data'][f'sensor_{sensor_index}'] = {
                        'index': sensor_index,
                        'type': 'Temperature',
                        'value': round(temp, 2),
                        'unit': 'C'
                    }
                    sensor_index += 1
                    
                    # Add humidity data
                    response['data'][f'sensor_{sensor_index}'] = {
                        'index': sensor_index,
                        'type': 'Humidity',
                        'value': round(humidity, 2),
                        'unit': '%'
                    }
                    sensor_index += 1
                    
                    # Add pressure data
                    response['data'][f'sensor_{sensor_index}'] = {
                        'index': sensor_index,
                        'type': 'Pressure',
                        'value': round(pressure, 2),
                        'unit': 'hPa'
                    }
                    sensor_index += 1
                    
                    # Add gas data
                    response['data'][f'sensor_{sensor_index}'] = {
                        'index': sensor_index,
                        'type': 'Gas',
                        'value': round(gas, 2),
                        'unit': 'Ω'
                    }
                    sensor_index += 1
                except Exception as e:
                    serial.write(f'Error reading BME680: {str(e)}\r\n'.encode('utf-8'))

            # Only read VL6180X sensor if available
            if available_sensors['vl6180x'] and vl6180x is not None:
                try:
                    range_mm = vl6180x.range
                    lux = vl6180x.read_lux(adafruit_vl6180x.ALS_GAIN_1)
                    
                    # Add distance data
                    response['data'][f'sensor_{sensor_index}'] = {
                        'index': sensor_index,
                        'type': 'Distance',
                        'value': range_mm,
                        'unit': 'mm'
                    }
                    sensor_index += 1
                    
                    # Add light data
                    response['data'][f'sensor_{sensor_index}'] = {
                        'index': sensor_index,
                        'type': 'Light',
                        'value': round(lux, 2),
                        'unit': 'lux'
                    }
                    sensor_index += 1
                except Exception as e:
                    serial.write(f'Error reading VL6180X: {str(e)}\r\n'.encode('utf-8'))

            # Only read NAU7802 ADC if available
            if available_sensors['nau7802'] and nau7802 is not None:
                try:
                    # Average of 5 readings
                    weight_sum = 0
                    samples = 5
                    samples_read = 0
                    for _ in range(samples):
                        if nau7802.available:
                            weight_sum += nau7802.read()
                            samples_read += 1
                            time.sleep(0.01)  # Short delay between readings
                    
                    if samples_read > 0:
                        weight_raw = weight_sum / samples_read
                        # Add ADC data
                        response['data'][f'sensor_{sensor_index}'] = {
                            'index': sensor_index,
                            'type': 'ADC',
                            'value': round(weight_raw, 2),
                            'unit': 'raw'
                        }
                        sensor_index += 1
                except Exception as e:
                    serial.write(f'Error reading NAU7802: {str(e)}\r\n'.encode('utf-8'))
                    
            # Only read HX711 ADC if available
            if available_sensors['hx711'] and channel_a is not None:
                try:
                    
                    raw_value = channel_a.value
                    signed_value = convert_signed_24bit(raw_value)
                    
                    # Add HX711 data
                    response['data'][f'sensor_{sensor_index}'] = {
                        'index': sensor_index,
                        'type': 'Weight',
                        'value': signed_value,
                        'unit': 'raw'
                    }
                    sensor_index += 1
                except Exception as e:
                    serial.write(f'Error reading HX711: {str(e)}\r\n'.encode('utf-8'))
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
