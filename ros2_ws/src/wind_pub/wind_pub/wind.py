import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
import serial

"""
Serial Message Order
--------------------
<STX> NodeAddr,WindDir,WindSpd,Unit,Status <ETX> CS
"""

class WindPubNode(Node):
    def __init__(self):
        # Init Node
        super().__init__("wind_node")
        self.node_namespace = "wind/"

        # Init serial handler
        self.ser = serial.Serial("/dev/FTDI_WIND", baudrate=38400)
        self.curr_string = ""
        self.ongoing_string = False

        # Init moving window
        self.window_size = 4
        self.dir_window = []
        self.speed_window = []

        # Create publishers
        self.wind_dir_pub = self.create_publisher(Int32, self.node_namespace + "dir", 10)
        self.wind_speed_pub = self.create_publisher(Float32, self.node_namespace + "speed", 10)
        self.wind_status_pub = self.create_publisher(Int32, self.node_namespace + "status", 10)

        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        while self.ser.in_waiting > 0:
            char = self.ser.read()

            # STX
            if char == b'\x02':
                self.curr_string = ""
                self.ongoing_string = True

            # ETX
            elif char == b'\x03':
                self.ongoing_string = False
                message = self.curr_string.split(',')[:-1]
                self.parse_message(message)

            elif self.ongoing_string:
                self.curr_string += str(char, encoding='utf-8')

    def parse_message(self, message):
        dir = int(message[1]) if message[1] != '' else None
        speed = float(message[2]) if message[2] != '' else None
        status = message[4]

        # Checksum
        if status == "00":
            self.update_window(dir, speed)
            self.publish_wind_data()

    def update_window(self, dir, speed):
        # Direction moving window
        if len(self.dir_window) < self.window_size:
            self.dir_window.append(dir)
        else:
            self.dir_window.pop(0)
            self.dir_window.append(dir)
        
        # Speed moving window
        if len(self.speed_window) < self.window_size:
            self.speed_window.append(speed)
        else:
            self.speed_window.pop(0)
            self.speed_window.append(speed)        

    def publish_wind_data(self):
        if len(self.dir_window != 0):
            dir_avg = sum(self.dir_window) / len(self.dir_window)
            dir_data = Int32(data=dir_avg)
            self.wind_dir_pub.publish(dir_data)
        
        if len(self.speed_window != 0):
            speed_avg = sum(self.speed_window) / len(self.speed_window)
            speed_data = Float32(data=speed_avg)
            self.wind_speed_pub.publish(speed_data)

def main(args=None):
    rclpy.init(args=args)
    node = WindPubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
