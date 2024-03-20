import serial

def parseMessage(message):
    id = message[0]
    dir = int(message[1]) if message[1] != '' else None
    speed = float(message[2]) if message[2] != '' else None
    unit = message[3]
    status = int(message[4])
    return (id, dir, speed, unit, status)

if __name__ == '__main__':
    # Open serial port
    ser = serial.Serial(port='COM3',\
                        baudrate=38400)
    curr_string = ""
    ongoing_string = False

    while True:
        if ser.inWaiting() > 0:
            char = ser.read()
            if char == b'\x02':
                curr_string = ""
                ongoing_string = True
            elif char == b'\x03':
                ongoing_string = False
                message = curr_string.split(',')[:-1]
                print(parseMessage(message))
                
            elif ongoing_string:
                curr_string += str(char, encoding='utf-8')