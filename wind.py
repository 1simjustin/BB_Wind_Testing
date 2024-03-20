import serial

"""
Serial Message Order
--------------------
<STX> NodeAddr,WindDir,WindSpd,Unit,Status <ETX> CS
"""
dir_window = []
speed_window = []
window_size = 4

def parse_message(self, message):
    dir = int(message[1]) if message[1] != '' else None
    speed = float(message[2]) if message[2] != '' else None
    status = message[4]

    # Checksum
    if status == "00":
        update_window(dir, speed)
        publish_wind_data()

def publish_wind_data(self):
    if len(dir_window) != 0:
        dir_avg = sum(dir_window) / len(dir_window)
        print(dir_avg)
    
    if len(speed_window) != 0:
        speed_avg = sum(speed_window) / len(speed_window)
        print(speed_avg)

def update_window(self, dir, speed):
    # Direction moving window
    if len(dir_window) < window_size:
        dir_window.append(dir)
    else:
        dir_window.pop(0)
        dir_window.append(dir)
    
    # Speed moving window
    if len(speed_window) < window_size:
        speed_window.append(speed)
    else:
        speed_window.pop(0)
        speed_window.append(speed)

def main(args=None):
    ser = serial.Serial("/dev/FTDI_WIND", baudrate=38400)
    curr_string = ""
    ongoing_string = False

    while True:
        char = ser.read()

        # STX
        if char == b'\x02':
            curr_string = ""
            ongoing_string = True

        # ETX
        elif char == b'\x03':
            ongoing_string = False
            message = curr_string.split(',')[:-1]
            parse_message(message)

        elif ongoing_string:
            curr_string += str(char, encoding='utf-8')

if __name__ == '__main__':
    main()
