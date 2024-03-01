# BB_Wind_Testing

This repository consists of test code for interfacing with the GILL wind sensor, converting it from RS232 messages to ros2 messages.

## Table of Contents

## Pin Diagram
| Pin Number | Cable Colour | Purpose | Connection |
|------------|--------------|---------|------------|
| 1          |Green         |Sig Gnd  |RS232 Pin5  |
| 2          |Orange        |V-       |GND         |
| 3          |Brown         |V+       |12V         |
| 4          |-             |-        |-           |
| 5          |Orange White  |TXD      |RS232 Pin2  |
| 6          |-             |-        |-           |
| 7          |Green White   |RXD      |RS232 Pin3  |
| 8          |-             |-        |-           |
| 9          |-             |-        |-           |

## Setup

### USB Rules

This helps to set a fixed port to read data from the wind sensor from to prevent dynamic allocation of USB ports from affecting wind sensor operation. Currently this only supports use of the FTDI RS232-USB converter (white and blue one).

Place the file `50-usb-serial.rules` into the folder `/etc/udev/rules.d/`. From this folder you can run the following command to do this:
```
sudo cp 50-usb-serial.rules ~/etc/udev/rules.d
```
Afterwards, run the following command to activate the rule. Alternatively you can reboot the computer.
The rule will automatically activate upon boot up in the future.
```
sudo udevadm trigger
```