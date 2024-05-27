import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range


class ConvertScanToRange(Node):

    def __init__(self):
        super().__init__('scan_to_range')
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.fl_range_sensor = self.create_publisher(Range, '/fl_range_sensor', 10)
        self.fr_range_sensor = self.create_publisher(Range, '/fr_range_sensor', 10)
        self.lidar_subscriber  

    def lidar_callback(self, msg):
        def getmin(a, b):
            in_rng = lambda x: msg.range_min <= x <= msg.range_max
            vsp = list(filter(in_rng, msg.ranges[a:b]))
            if len(vsp) > 0:
                return min(vsp)
            else:
                return sys.maxsize

        newfront = min(getmin(330, 360), getmin(0, 30))
        newleft = getmin(25, 35)
        newright = getmin(325, 335)
        new_backleft = getmin(90, 100)
        new_backright = getmin(260, 270)

        fl_sensor_msg = Range()
        fl_sensor_msg.header.stamp = msg.header.stamp
        fl_sensor_msg.header.frame_id = 'fl_sensor_link'
        fl_sensor_msg.field_of_view = 0.174532925
        fl_sensor_msg.min_range = msg.range_min
        fl_sensor_msg.max_range = msg.range_max
        fl_sensor_msg.range = float(newleft)
        self.fl_range_sensor.publish(fl_sensor_msg)

        fr_sensor_msg = Range()
        fr_sensor_msg.header.stamp = msg.header.stamp
        fr_sensor_msg.header.frame_id = 'fr_sensor_link'
        fr_sensor_msg.field_of_view = 0.174532925
        fr_sensor_msg.min_range = msg.range_min
        fr_sensor_msg.max_range = msg.range_max
        fr_sensor_msg.range = float(newright)
        self.fr_range_sensor.publish(fr_sensor_msg)

def main(args=None):
    rclpy.init(args=args)
    conver_scan_to_range = ConvertScanToRange()
    rclpy.spin(conver_scan_to_range)
    conver_scan_to_range.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
