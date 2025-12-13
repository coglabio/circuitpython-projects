# Dual Motor Controller for Pimoroni Tiny2350

A CircuitPython-based dual H-bridge motor controller with analog and digital inputs for the Pimoroni Tiny2350 development board.

## Features

- **Dual Motor Control**: Independent control of two DC motors via H-bridge drivers
- **PWM Speed Control**: Variable speed control (0-100%) for each motor
- **Bidirectional Control**: Clockwise (CW) and counter-clockwise (CCW) direction control
- **4 Analog Inputs**: Read voltage from A0-A3 (GP26-GP29)
- **4 Digital Inputs**: Read digital states from GP0-GP3 with configurable pull resistors
- **Serial Communication**: USB serial interface for command and control
- **Non-Volatile Storage**: Device ID and pull resistor configuration stored in NVM
- **JSON Status Output**: Query all sensor states, motor status, and configuration

## Hardware Requirements

- **Pimoroni Tiny2350** development board
- **Two H-Bridge Motor Controllers** (e.g., L298N, TB6612FNG, DRV8833)
- **DC Motors** (2x)
- **Power Supply** appropriate for your motors
- **Optional**: Analog sensors, digital sensors/switches

## CircuitPython Setup

### 1. Download CircuitPython

Download the latest CircuitPython UF2 file for the Tiny2350:
- Visit: https://circuitpython.org/board/pimoroni_tiny2350/
- Download the latest stable release (CircuitPython 10.0.3 or newer)

### 2. Install CircuitPython

1. **Enter Bootloader Mode**:
   - Hold down the **BOOT** button on the Tiny2350
   - While holding BOOT, press and release the **RESET** button
   - Continue holding BOOT until a drive named **RPI-RP2** appears on your computer

2. **Install the Firmware**:
   - Drag the downloaded `.uf2` file onto the **RPI-RP2** drive
   - The board will automatically reboot
   - A new drive named **CIRCUITPY** will appear

3. **Copy the Code**:
   - Copy `code.py` to the **CIRCUITPY** drive
   - The code will run automatically on boot

## Pin Connections

### Motor 1 (H-Bridge 1)
| Tiny2350 Pin | H-Bridge Pin | Description |
|--------------|--------------|-------------|
| **GP6** | IN1 | Motor 1 direction control 1 |
| **GP7** | IN2 | Motor 1 direction control 2 |

### Motor 2 (H-Bridge 2)
| Tiny2350 Pin | H-Bridge Pin | Description |
|--------------|--------------|-------------|
| **GP4** | IN1 | Motor 2 direction control 1 |
| **GP5** | IN2 | Motor 2 direction control 2 |

### Analog Inputs
| Tiny2350 Pin | Description |
|--------------|-------------|
| **A0 (GP26)** | Analog input channel 0 (0-3.3V) |
| **A1 (GP27)** | Analog input channel 1 (0-3.3V) |
| **A2 (GP28)** | Analog input channel 2 (0-3.3V) |
| **A3 (GP29)** | Analog input channel 3 (0-3.3V) |

### Digital Inputs
| Tiny2350 Pin | Description |
|--------------|-------------|
| **GP0** | Digital input channel 0 (configurable pull-up/down) |
| **GP1** | Digital input channel 1 (configurable pull-up/down) |
| **GP2** | Digital input channel 2 (configurable pull-up/down) |
| **GP3** | Digital input channel 3 (configurable pull-up/down) |

### Power
| Tiny2350 Pin | Connection |
|--------------|------------|
| **3V3** | 3.3V output for sensors (max 300mA) |
| **GND** | Common ground |
| **VBUS** | 5V USB power output |

## H-Bridge Wiring Notes

### Typical H-Bridge Connection
```
Tiny2350          H-Bridge          Motor
--------          --------          -----
GP6 (IN1) -----> IN1
GP7 (IN2) -----> IN2
                  OUT1 ----------> Motor +
                  OUT2 ----------> Motor -
                  VCC  <---------- Motor Power Supply +
                  GND  <---------- Motor Power Supply - & Tiny2350 GND
```

**Important**: 
- Connect the motor power supply ground to the Tiny2350 ground (common ground)
- Do NOT power motors from the Tiny2350's 3.3V or VBUS pins
- Use an external power supply rated for your motors
- Most H-bridges require a logic voltage connection (connect to Tiny2350 3V3)

## Serial Commands

Connect to the Tiny2350 via USB serial (115200 baud, or default) and send commands terminated with carriage return (`\r`).

### Motor Control Commands

| Command | Description | Example |
|---------|-------------|---------|
| `M1CW` | Set Motor 1 direction to clockwise | `M1CW` |
| `M1CCW` | Set Motor 1 direction to counter-clockwise | `M1CCW` |
| `M1ON` | Turn Motor 1 ON | `M1ON` |
| `M1OFF` | Turn Motor 1 OFF | `M1OFF` |
| `M1S[speed]` | Set Motor 1 speed (0.0-100.0) | `M1S75.5` |
| `M2CW` | Set Motor 2 direction to clockwise | `M2CW` |
| `M2CCW` | Set Motor 2 direction to counter-clockwise | `M2CCW` |
| `M2ON` | Turn Motor 2 ON | `M2ON` |
| `M2OFF` | Turn Motor 2 OFF | `M2OFF` |
| `M2S[speed]` | Set Motor 2 speed (0.0-100.0) | `M2S50.0` |

### Configuration Commands

| Command | Description | Example |
|---------|-------------|---------|
| `PULLUP` | Set all digital inputs to pull-up mode | `PULLUP` |
| `PULLDOWN` | Set all digital inputs to pull-down mode | `PULLDOWN` |
| `FLOAT` | Set all digital inputs to floating (no pull) | `FLOAT` |
| `setid [id]` | Set device identifier | `setid ROBOT-01` |
| `getid` | Get device identifier (JSON response) | `getid` |
| `?` | Query all sensors and motor status (JSON) | `?` |

### Example Usage

```
M1S50.0          # Set motor 1 speed to 50%
M1CW             # Set motor 1 to clockwise
M1ON             # Turn motor 1 on
M2S75.0          # Set motor 2 speed to 75%
M2CCW            # Set motor 2 to counter-clockwise
M2ON             # Turn motor 2 on
?                # Query status
M1OFF            # Turn motor 1 off
M2OFF            # Turn motor 2 off
PULLDOWN         # Change digital inputs to pull-down
```

## JSON Status Response

Send `?` to receive a JSON response with all sensor readings and motor status:

```json
{
  "id": "device-uuid-here",
  "data": {
    "analog_0": {
      "index": 0,
      "type": "Voltage",
      "channel": 0,
      "value": 3.142,
      "unit": "V",
      "pin": "A0"
    },
    "analog_1": { ... },
    "analog_2": { ... },
    "analog_3": { ... },
    "digital_0": {
      "index": 4,
      "type": "Digital",
      "channel": 0,
      "value": true,
      "pin": "GP0"
    },
    "digital_1": { ... },
    "digital_2": { ... },
    "digital_3": { ... },
    "motor_1": {
      "index": 8,
      "type": "Motor",
      "motor": 1,
      "speed": 50.0,
      "direction": "CW",
      "running": true
    },
    "motor_2": {
      "index": 9,
      "type": "Motor",
      "motor": 2,
      "speed": 75.0,
      "direction": "CCW",
      "running": true
    },
    "config": {
      "pull_resistor": "UP"
    }
  }
}
```

## Pull Resistor Configuration

Digital inputs can be configured with three different modes:

- **Pull-Up (default)**: Pin reads HIGH when floating, LOW when connected to ground
  - Use for switches that connect to ground
  - Internal resistor pulls pin to 3.3V
  - Command: `PULLUP`

- **Pull-Down**: Pin reads LOW when floating, HIGH when connected to 3.3V
  - Use for switches that connect to 3.3V
  - Internal resistor pulls pin to ground
  - Command: `PULLDOWN`

- **Floating**: No internal pull resistor
  - Pin state is undefined when not driven
  - Use with external pull resistors or push-pull outputs
  - Command: `FLOAT`

The configuration is stored in non-volatile memory and persists across power cycles.

## Technical Specifications

### Pimoroni Tiny2350
- **MCU**: RP2350A (Dual ARM Cortex M33 @ 150MHz)
- **RAM**: 520KB SRAM
- **Flash**: 4MB
- **GPIO**: 12 pins available
- **ADC**: 4 channels, 12-bit resolution
- **PWM**: All GPIO pins support PWM
- **USB**: USB-C for power and programming
- **Voltage**: 3V - 5.5V input, 3.3V logic

### PWM Configuration
- **Frequency**: 5000 Hz (5 kHz)
- **Resolution**: 16-bit (0-65535)
- **Duty Cycle**: 0-100% (mapped from speed percentage)

## Troubleshooting

### Motors not responding
- Check H-bridge connections (IN1, IN2 pins)
- Verify motor power supply is connected and adequate
- Ensure common ground between Tiny2350 and motor power supply
- Check that motors are turned ON with `M1ON` or `M2ON`

### Digital inputs always read the same value
- Check pull resistor configuration (`PULLUP` vs `PULLDOWN`)
- Verify sensor/switch connections
- Test with a jumper wire to ground (pull-up) or 3.3V (pull-down)

### Analog inputs read 0V or 3.3V
- Check sensor connections
- Verify sensor power supply (3.3V from Tiny2350)
- Ensure analog signal is within 0-3.3V range

### CircuitPython drive not appearing
- Try entering bootloader mode again
- Use a different USB cable (some are charge-only)
- Try a different USB port

## License

Copyright (C) 2025 - Lab Sync Inc.
All rights reserved.

## Version

Version: 1.0
