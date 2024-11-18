import rclpy
from rclpy.node import Node
import v2x_cohdainterfaces.msg as v2xmsg
import v2x_logic.models as models
import nav_msgs.msg as navmsg
import sensor_msgs.msg as sensormsg

class MAP_Handler(Node):
    def __init__(self) -> None:
        super().__init__("MAP_Subscriber")
        self.subscription = self.create_subscription(v2xmsg.Mapem, "Cohda_Signals/MAPEM", self.listener_callback, 10)
        self.subscription

        self.Map: dict[int, models.Intersection] = dict()

        self.path_publishers = self.create_publisher(navmsg.Path, "Cohda_Signals/Intersection", 10)


    def listener_callback(self, msg):
        interstections: v2xmsg.Intersectiongeometrylist = msg.map.intersections
        for intersection in interstections.intersectiongeometrylist:
            intersection: v2xmsg.Intersectiongeometry
            intersectionID: int = intersection.id.id.intersectionid
            print(intersectionID)

            self.Map[intersectionID] = models.Intersection(intersection)
            print(self.Map[intersectionID])

class GPS_Handler(Node):
    def __init__(self) -> None:
        super().__init__("GPS_Subscriber")
        self.subscription = self.create_subscription(v2xmsg.Gps, "Cohda_Signals/GPS", self.listener_callback, 10)
        self.subscription

        self.GPS = {
            "latitude": None,
            "longitude": None,
            "altitude": None
        }

    def listener_callback(self, msg: sensormsg.NavSatFix):
        self.GPS["latitude"] = msg.latitude
        self.GPS["longitude"] = msg.longitude
        self.GPS["altitude"] = msg.altitude


def main(args=None):
    rclpy.init(args=args)
    
    MAP_Data = dict()
    GPS_history = list()
    
    map_data_listener = MAP_Handler()
    gps_data_listener = GPS_Handler()
    
    while rclpy.ok():
        rclpy.spin_once(map_data_listener)
        rclpy.spin_once(gps_data_listener)

        GPS_history.append(gps_data_listener.GPS)

        if len(GPS_history) > 16:
            del GPS_history[0]

    map_data_listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
