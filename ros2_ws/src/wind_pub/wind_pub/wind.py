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
        ser = serial.Serial("/dev/FTDI_WIND", baudrate=9600)
        curr_string = ""
        ongoing_string = False

        # Create publishers

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
