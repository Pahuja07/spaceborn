import random
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import Int32

class SignalJammer(Node):

    def __init__(self):
        super().__init__('signal_jammer')

        self.subscription = self.create_subscription(
            NavSatFix,
            '/a200_0000/sensors/gps_0/fix',
            self.gps_callback,
            10
        )

        self.publisher_ = self.create_publisher(
            NavSatFix,
            '/a200_0000/sensors/gps_0/fix_jammed',
            10
        )
        self.mode_subscription = self.create_subscription(
            Int32,
            '/jammer_mode',
            self.mode_callback,
            10
        )
        
        self.declare_parameter("jammer_mode", 0)
        self.declare_parameter("noise_level", 0.00005)
        self.declare_parameter("drift_speed", 0.00001)
        self.declare_parameter("spoof_latitude", 19.0760)
        self.declare_parameter("spoof_longitude", 72.8777)
        self.declare_parameter("spoof_altitude", 20.0)
        self.last_msg = None
        self.drift_lat = 0.0
        self.drift_lon = 0.0
        self.jammer_mode = self.get_parameter(
            "jammer_mode"
        ).value

        self.noise_level = self.get_parameter(
            "noise_level"
        ).value

        self.drift_speed = self.get_parameter(
            "drift_speed"
        ).value

        self.spoof_latitude = self.get_parameter(
            "spoof_latitude"
        ).value

        self.spoof_longitude = self.get_parameter(
            "spoof_longitude"
        ).value

        self.spoof_altitude = self.get_parameter(
            "spoof_altitude"
        ).value
    def mode_callback(self, msg):

        self.jammer_mode = msg.data

        self.get_logger().info(
            f"Jammer Mode Changed to {self.jammer_mode}"
        )  
       
    def gps_callback(self, msg):
    
        self.noise_level = self.get_parameter("noise_level").value
        self.drift_speed = self.get_parameter("drift_speed").value
        self.spoof_latitude = self.get_parameter("spoof_latitude").value
        self.spoof_longitude = self.get_parameter("spoof_longitude").value
        self.spoof_altitude = self.get_parameter("spoof_altitude").value
        
        if self.jammer_mode == 0:
             
              self.get_logger().info("Normal GPS")
              
        elif self.jammer_mode == 1:
        
              msg.status.status = NavSatStatus.STATUS_NO_FIX
              self.get_logger().info("GPS Lost")
              
        elif self.jammer_mode == 2:
        
              noise = random.uniform(
                  -self.noise_level,
                   self.noise_level
              )
              
              msg.latitude += noise
              msg.longitude += noise
              
              self.get_logger().info("GPS Noise Added") 
              
              
        elif self.jammer_mode == 3:

              self.drift_lat += self.drift_speed
              self.drift_lon += self.drift_speed
              msg.latitude += self.drift_lat
              msg.longitude += self.drift_lon

              self.get_logger().info("GPS Drift Added")
              
        elif self.jammer_mode == 4:

              if self.last_msg is None:
                  self.last_msg = msg

              msg = self.last_msg

              self.get_logger().info("GPS Frozen")
              
        elif self.jammer_mode == 5:

              msg.latitude = self.spoof_latitude
              msg.longitude = self.spoof_longitude
              msg.altitude = self.spoof_altitude

              self.get_logger().info("GPS Spoofed")
              
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = SignalJammer()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
