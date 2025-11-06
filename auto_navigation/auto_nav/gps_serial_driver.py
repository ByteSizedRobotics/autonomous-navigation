import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial
import pynmea2  

class GPSSerialNode(Node):
    def __init__(self):
        super().__init__('gps_serial_node')
        self.pub = self.create_publisher(NavSatFix, '/fix', 10)
        port = self.declare_parameter('port', '/dev/GPS_serial').value
        baud = self.declare_parameter('baudrate', 9600).value
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Opened {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open GPS serial: {e}")
            self.ser = None
        self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if not self.ser or self.ser.in_waiting == 0:
            return
        line = self.ser.readline().decode(errors='ignore').strip()
        if not line:
            return
        try:
            msg = pynmea2.parse(line)
            if isinstance(msg, pynmea2.types.talker.GGA) or isinstance(msg, pynmea2.types.talker.RMC):
                nav = NavSatFix()
                nav.header.stamp = self.get_clock().now().to_msg()
                nav.header.frame_id = 'gps'  # or base_link if you want
                nav.status.status = NavSatStatus.STATUS_FIX if getattr(msg, 'gps_qual', None) else NavSatStatus.STATUS_NO_FIX
                # fill lat/lon using msg.latitude/msg.longitude helper (pynmea2 provides .latitude/.longitude)
                nav.latitude = getattr(msg, 'latitude', 0.0)
                nav.longitude = getattr(msg, 'longitude', 0.0)
                nav.altitude = getattr(msg, 'altitude', 0.0)
                self.pub.publish(nav)
        except Exception as e:
            self.get_logger().debug(f"Failed to parse NMEA: {e}")
