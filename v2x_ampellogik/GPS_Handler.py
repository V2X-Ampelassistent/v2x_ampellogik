import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

class GPS_Handler(Node):
    def __init__(self):
        super().__init__('GPS_Handler')
        self.subscription = self.create_subscription(
            NavSatFix,
            'Cohda_Signals/GPS',
            self.listener_callback,
            10
        )
        self.subscription

        self.gps_vis_pub = self.create_publisher(Marker, 'GPS_Pos', 10)
    def listener_callback(self, msg: NavSatFix):
        self.get_logger().info(f'lat: "{msg.latitude}"')
        self.get_logger().info(f'lon: "{msg.longitude}"')
        self.get_logger().info(f'altitude: "{msg.altitude}"')

        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = msg.latitude
        marker.pose.position.y = msg.longitude
        marker.pose.position.z = msg.altitude
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.gps_vis_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    gps_data_listener = GPS_Handler()
    rclpy.spin(gps_data_listener)
    gps_data_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
