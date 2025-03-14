# HLK-LD2450 Basic Operation Example

This example demonstrates the basic usage of the HLK-LD2450 radar driver component.

## Functionality

This application:

1. Initializes the LD2450 radar driver with default settings
2. Registers a callback to receive radar data
3. Configures the radar module (firmware version, tracking mode)
4. Displays detected targets' information every 3 seconds

## Hardware Required

* ESP32 development board
* HLK-LD2450 radar module
* Jumper wires to connect them

## Connection

Connect the HLK-LD2450 to your ESP32 board as follows:

| HLK-LD2450 Pin | ESP32 Pin (Default) |
|----------------|---------------------|
| TX             | GPIO16              |
| RX             | GPIO17              |
| 5V             | 5V                  |
| GND            | GND                 |

## Configuration

You can modify the UART pins and other settings in the `app_main()` function:

```c
ld2450_config_t config = LD2450_DEFAULT_CONFIG();
config.uart_port = UART_NUM_1;     // Change UART port
config.uart_rx_pin = 5;            // Change RX pin
config.uart_tx_pin = 6;            // Change TX pin 
config.uart_baud_rate = 115200;    // Change baud rate
```

## Building and Running

Run the following commands:

```bash
idf.py build
idf.py -p PORT flash monitor
```

Replace `PORT` with your serial port (e.g., `/dev/ttyUSB0`, `COM3`).

## Expected Output

```
I (332) ld2450_example: HLK-LD2450 Basic Example
I (332) ld2450_example: Initializing LD2450 driver...
I (352) LD2450: LD2450 driver initialized on UART2 (RX: GPIO16, TX: GPIO17, baud: 256000)
I (352) LD2450: Auto-processing enabled with task priority 5
I (1352) ld2450_example: Radar firmware version: V1.02.22062416
I (1392) ld2450_example: Current tracking mode: Multi Target
I (1432) ld2450_example: Set tracking mode to Multi Target
I (4432) ld2450_example: ----- Radar Data -----
I (4432) ld2450_example: Detected 1 targets:
I (4432) ld2450_example: Target 1:
I (4432) ld2450_example:   Position: X=245 mm, Y=1356 mm
I (4442) ld2450_example:   Polar: Distance=1378.81 mm, Angle=-10.25°
I (4452) ld2450_example:   Speed: 5 cm/s
I (4452) ld2450_example:   Resolution: 78 mm
```