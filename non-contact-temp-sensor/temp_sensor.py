import serial
import time
import struct
import numpy as np
import matplotlib.pyplot as plt
plt.ion()  # Enable interactive mode

# Configure the serial connection
SERIAL_PORT = "/dev/tty.usbserial-0001"  # Use the correct serial port for your Raspberry Pi
BAUD_RATE = 115200

def calculate_temperature(msb, lsb):
    """ Convert 2-byte signed temperature data into Celsius """
    raw_value = (msb << 8) | lsb  # Combine MSB and LSB
    if raw_value & 0x8000:  # Check if negative (2's complement)
        raw_value -= 0x10000
    return raw_value * 0.1  # Convert to Celsius

def send_request(ser, start_address=0, num_registers=1025):
    """ Send request command to the sensor """
    if start_address < 0 or start_address > 1024:
        raise ValueError("Start address must be between 0 and 1024")
    if num_registers < 1 or num_registers > (1025 - start_address):
        raise ValueError("Invalid number of registers")

    # Construct the request frame
    request_frame = bytearray([
        0x11,  # START
        (start_address >> 8) & 0x07,  # Start Address MSB (only 3 bits used)
        start_address & 0xFF,  # Start Address LSB
        (num_registers >> 8) & 0x07,  # Number of Registers MSB (only 3 bits used)
        num_registers & 0xFF,  # Number of Registers LSB
        0x98  # END
    ])

    # Send request
    ser.write(request_frame)
    time.sleep(0.5)  # Wait for response

def read_response(ser, num_registers):
    """ Read response from the sensor """
    response_size = (num_registers * 2) + 4  # Calculate expected response size
    response = ser.read(response_size)

    if len(response) != response_size:
        raise ValueError(f"Incomplete response received: {len(response)} bytes instead of {response_size}")

    # Validate start and end bytes
    if response[:2] != b'\x16\x98' or response[-2:] != b'\x1A\x9C':
        raise ValueError("Invalid response format")

    # Extract temperature values
    temperatures = []
    for i in range(num_registers):
        msb = response[2 + (i * 2)]
        lsb = response[3 + (i * 2)]
        temperatures.append(calculate_temperature(msb, lsb))

    return temperatures

def main():
    """ Main function to communicate with the temperature sensor """
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print("Starting continuous temperature monitoring...")
            print("Press Ctrl+C to exit")
            
            while True:
                # get ambient temp (register 0)
                send_request(ser, start_address=0, num_registers=1)
                ambient_temp = read_response(ser, num_registers=1)

                print(f"\nAmbient temperature: {ambient_temp[0]:.1f} °C")

                # read all 1024 pixel temps
                send_request(ser, start_address=1, num_registers=1024)
                temperatures = read_response(ser, num_registers=1024)
                
                # calculate average temp
                avg_temp = sum(temperatures) / len(temperatures)
                print(f"\nAverage temperature: {avg_temp:.1f} °C")
                
                # Reshape temperatures into 32x32 grid and display heatmap
                temp_grid = np.array(temperatures).reshape(32, 32)
                
                plt.clf()  # Clear the current figure
                plt.imshow(temp_grid, cmap='hot', interpolation='nearest')
                plt.colorbar(label='Temperature (°C)')
                plt.title(f'Temperature Heatmap\nAmbient: {ambient_temp[0]:.1f}°C, Avg: {avg_temp:.1f}°C')
                plt.draw()
                plt.pause(0.1)  # Small pause to update the plot
                
                time.sleep(0.5)  # Reduced sleep time for more responsive updates

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except ValueError as e:
        print(f"Value error: {e}")
    except KeyboardInterrupt:
        print("\nStopping temperature monitoring...")

if __name__ == "__main__":
    main()