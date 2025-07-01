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
import os
from adafruit_hx711.hx711 import HX711
from adafruit_hx711.analog_in import AnalogIn
# Import DS2484 (I2C to 1-Wire adapter)
from adafruit_ds248x import Adafruit_DS248x

serial = usb_cdc.console

# Initialize I2C bus and sensors
sda = board.SDA
scl = board.SCL
i2c = None

def generate_uuid4():
    # Get 16 random bytes
    random_bytes = bytearray(os.urandom(16))

    # Set version to 4 => xxxx => 0100
    random_bytes[6] = (random_bytes[6] & 0x0F) | 0x40

    # Set variant to RFC 4122 => 10xx
    random_bytes[8] = (random_bytes[8] & 0x3F) | 0x80

    # Format into UUID string
    uuid_str = (
        "myrandomUUID"
        # f"{random_bytes[0]:02x}{random_bytes[1]:02x}{random_bytes[2]:02x}{random_bytes[3]:02x}-"
        # f"{random_bytes[4]:02x}{random_bytes[5]:02x}-"
        # f"{random_bytes[6]:02x}{random_bytes[7]:02x}-"
        # f"{random_bytes[8]:02x}{random_bytes[9]:02x}-"
        # f"{random_bytes[10]:02x}{random_bytes[11]:02x}{random_bytes[12]:02x}"
        # f"{random_bytes[13]:02x}{random_bytes[14]:02x}{random_bytes[15]:02x}"
    )

    return uuid_str

def zero_channel(channel):
    """Initiate internal calibration for current channel.Use when scale is started,
    a new channel is selected, or to adjust for measurement drift. Remove weight
    and tare from load cell before executing."""
    print(
        "channel {0:1d} calibrate.INTERNAL: {1:5s}".format(
           channel, str(nau7802.calibrate("INTERNAL"))
        )
    )
    print(
        "channel {0:1d} calibrate.OFFSET:   {1:5s}".format(
            channel, str(nau7802.calibrate("OFFSET"))
        )
    )
    print(f"...channel {channel:1d} zeroed")

# Initialize sensors individually and track which ones are available
available_sensors = {
    'bme680': False,
    'vl6180x': False,
    'nau7802': False,
    'hx711': False,
    'ds18b20': False
}

# Initialize BME680 sensor (temperature, humidity, pressure, gas)
bme680 = None
serial.write(f'Initializing the sensor...1\r\n'.encode('utf-8'))
try:
    if i2c is None:
        i2c = i2c = busio.I2C(scl, sda)
    bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c)
    available_sensors['bme680'] = True
    serial.write(f'BME680 sensor detected\r\n'.encode('utf-8'))
except Exception as e:
    serial.write(f'BME680 sensor not available: {str(e)}\r\n'.encode('utf-8'))

# Initialize VL6180X sensor (distance, light)
vl6180x = None
try:
    if i2c is None:
        i2c = i2c = busio.I2C(scl, sda)
    vl6180x = adafruit_vl6180x.VL6180X(i2c)
    available_sensors['vl6180x'] = True
    serial.write(f'VL6180X sensor detected\r\n'.encode('utf-8'))
except Exception as e:
    serial.write(f'VL6180X sensor not available: {str(e)}\r\n'.encode('utf-8'))

# Initialize NAU7802 ADC (weight)
nau7802 = None
try:
    if i2c is None:
        i2c = i2c = busio.I2C(scl, sda)
    nau7802 = cedargrove_nau7802.NAU7802(i2c)
    # Enable channel 1 for weight readings
    nau7802.channel = 1
    nau7802.gain = 128  # High gain for small load cells
    available_sensors['nau7802'] = True
    serial.write(f'NAU7802 sensor detected\r\n'.encode('utf-8'))
    serial.write(f'Calibrating the sensor...\r\n'.encode('utf-8'))
    zero_channel(1)  # Calibrate and zero channel
    serial.write(f'Sensor calibrated\r\n'.encode('utf-8'))
    serial.flush()
except Exception as e:
    serial.write(f'NAU7802 sensor not available: {str(e)}\r\n'.encode('utf-8'))

# Initialize HX711 load cell ADC
hx711 = None
channel_a = None
try:
    # Set up HX711 pins - using direct digital pins, not I2C
    data = digitalio.DigitalInOut(sda)
    data.direction = digitalio.Direction.INPUT
    clock = digitalio.DigitalInOut(scl)
    clock.direction = digitalio.Direction.OUTPUT

    # Initialize HX711 with direct pin connections
    hx711 = HX711(data, clock)
    channel_a = AnalogIn(hx711, HX711.CHAN_A_GAIN_128)
    available_sensors['hx711'] = True
    serial.write(f'HX711 sensor detected\r\n'.encode('utf-8'))
except Exception as e:
    serial.write(f'HX711 sensor not available: {str(e)}\r\n'.encode('utf-8'))

# Initialize DS2484 I2C to 1-Wire adapter and DS18B20 temperature sensors
ds2484 = None
ds18b20_roms = []
try:
    if i2c is None:
        i2c = busio.I2C(scl, sda)
    # Initialize the DS2484 adapter
    ds2484 = Adafruit_DS248x(i2c)
    
    # Scan for DS18B20 devices on the 1-Wire bus
    rom = bytearray(8)
    found_devices = 0
    
    # Search for up to 5 devices (adjust as needed)
    for _ in range(5):
        if not ds2484.onewire_search(rom):
            break
        # Store a copy of the ROM for each device found
        rom_copy = bytearray(8)
        for i in range(8):
            rom_copy[i] = rom[i]
        ds18b20_roms.append(rom_copy)
        found_devices += 1
    
    if found_devices > 0:
        available_sensors['ds18b20'] = True
        serial.write(f'Found {found_devices} DS18B20 temperature sensors\r\n'.encode('utf-8'))
    else:
        serial.write(f'No DS18B20 temperature sensors found\r\n'.encode('utf-8'))
except Exception as e:
    serial.write(f'DS2484/DS18B20 not available: {str(e)}\r\n'.encode('utf-8'))

id = ""
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
        return ""

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

def read_until(terminator=b'\r', timeout=0.5):
    """Read bytes from serial until terminator or timeout.
    
    This simplified implementation is designed to be compatible with device-hub
    while still being robust against KeyboardInterrupt errors.
    
    Args:
        terminator: Byte sequence that marks the end of input
        timeout: Maximum time to wait for input in seconds
        
    Returns:
        Bytes read up to (but not including) the terminator
    """
    buffer = b""
    start_time = time.monotonic()
    
    try:
        while (time.monotonic() - start_time) < timeout:
            # Check for data without blocking
            if serial.in_waiting > 0:
                char = serial.read(1)
                if char == terminator:
                    # If terminator is \r, also consume a following \n if present
                    if terminator == b'\r' and serial.in_waiting > 0:
                        next_char = serial.read(1)
                        if next_char != b'\n':
                            # If it's not \n, put it back in the buffer
                            serial.reset_input_buffer()
                            serial.write(next_char)
                    return buffer
                buffer += char
            else:
                # Use the shortest possible sleep to reduce chance of KeyboardInterrupt
                # but still avoid CPU hogging
                try:
                    time.sleep(0.001)  # Reduced from 0.01 to 0.001
                except (Exception, KeyboardInterrupt):
                    # Explicitly catch KeyboardInterrupt and continue
                    # This prevents the device from crashing
                    pass
    except (Exception, KeyboardInterrupt) as e:
        # Log any errors but don't crash
        try:
            if not isinstance(e, KeyboardInterrupt):
                serial.write(f"Error in read_until: {str(e)}\r\n".encode('utf-8'))
        except Exception:
            # If we can't even report the error, just continue silently
            pass
    
    # Return what we have if timeout occurs or on exception
    return buffer

def read_adc():
    """Read ADC value consistently for both manual and automated queries."""
    # Try to read from NAU7802 first if available
    if available_sensors['nau7802'] and nau7802 is not None:
        try:
            # Average of 5 readings for stability
            weight_sum = 0
            samples = 5
            samples_read = 0
            for _ in range(samples):
                if nau7802.available:
                    weight_sum += nau7802.read()
                    samples_read += 1
                    time.sleep(0.01)  # Short delay between readings
            
            if samples_read > 0:
                return round(weight_sum / samples_read, 1)
        except Exception:
            pass
    
    # Fall back to HX711 if NAU7802 failed or not available
    if available_sensors['hx711'] and channel_a is not None:
        try:
            raw_value = channel_a.value
            signed_value = convert_signed_24bit(raw_value)
            return signed_value
        except Exception:
            pass
    
    # Return a random value for testing if no ADC is available
    return round(random.uniform(200, 800), 1)

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

# 🚀 Indicate the sensor is ready to receive commands
serial.write(f'READY\r\n'.encode('utf-8'))
serial.flush()

while True:
    if serial.in_waiting > 0:
        # Read the full command line with timeout
        command_bytes = read_until(b'\r', timeout=2.0)
        
        # Only process if we received something
        if command_bytes:
            try:
                # Check if this is a ping request
                if command_bytes == b"PING":
                    # Send pong response
                    serial.write(b"PONG\r\n")
                    serial.flush()
                    continue  # Continue waiting for the next command
                
                try:
                    command_line = command_bytes.decode('utf-8')
                    # Split command and potential arguments
                    parts = command_line.split(' ', 1)
                    command = parts[0].strip()
                    args = parts[1].strip() if len(parts) > 1 else ''
                except Exception as e:
                    # Handle decoding errors or malformed commands
                    serial.write(f"error: malformed command: {repr(command_bytes)}\r\n".encode('utf-8'))
                    continue  # Skip to next command
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
                            response["data"][f"sensor_{sensor_index}"] = {
                                "index": sensor_index,
                                "type": "Light",
                                "value": round(lux, 2),
                                "unit": "lux"
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
                    
                    # Only read DS18B20 temperature sensors if available
                    if available_sensors['ds18b20'] and ds18b20_roms:
                        try:
                            # Read temperatures from all detected sensors
                            for rom in ds18b20_roms:
                                temperature = ds2484.ds18b20_temperature(rom)
                                
                                # Add temperature data for each sensor
                                response['data'][f'sensor_{sensor_index}'] = {
                                    'index': sensor_index,
                                    'type': 'Temperature',
                                    'value': round(temperature, 2),
                                    'unit': 'C'
                                }
                                sensor_index += 1
                        except Exception as e:
                            serial.write(f'Error reading DS18B20: {str(e)}\r\n'.encode('utf-8'))
                    
                    # If no sensors were found, add a default ADC reading
                    if len(response['data']) == 0:
                        response['data']['sensor_0'] = {
                            'unit': 'raw',
                            'type': 'ADC',
                            'value': read_adc(),
                            'index': 0
                        }
                    
                    # Send the response with proper CRLF line endings
                    serial.write(f'{json.dumps(response)}\r\n'.encode('utf-8'))
                    serial.flush()  # Ensure data is sent immediately
                    
                elif command == 'setid':
                    # Set device identifier
                    if args:
                        success = write_identifier(args)
                        if success:
                            id = args
                            serial.write(f"ok: id set to {id}\r\n".encode('utf-8'))
                        else:
                            serial.write(b"error: failed to set id\r\n")
                    else:
                        serial.write(b"error: missing id argument\r\n")
                elif command == 'getid':
                    # Get device identifier
                    serial.write(f"id: {id if id else 'not set'}\r\n".encode('utf-8'))
                elif command == 'ping':
                    # Handle text-based ping as well
                    serial.write(b"pong\r\n")
                    serial.flush()
                elif command == 'zero':
                    zero_channel(1)  # Calibrate and zero channel
                else:
                    serial.write(f"error: invalid command '{command}'\r\n".encode('utf-8'))
            except Exception as e:
                # Log any errors but don't crash
                try:
                    serial.write(f"Error: {str(e)}\r\n".encode('utf-8'))
                except Exception:
                    pass
            
            # Clear input buffer after processing each command to prevent backlog
            serial.reset_input_buffer()
    time.sleep(0.01)  # Small delay to prevent CPU hogging
    # This prevents CPU overuse when there's no data to process
    time.sleep(0.01)
