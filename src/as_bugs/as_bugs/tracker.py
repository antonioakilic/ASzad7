import rclpy
import time
import threading
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class Tracker(Node):
    def __init__(self):
        super().__init__('tracker')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.location_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', 1)
        self.pose_publisher = self.create_publisher(PoseStamped, '/robot_pose', 1)
        self.path_publisher = self.create_publisher(Path, '/path', 1)
        self.timer = self.create_timer(0.5, self.publish_markers)
        self.path_msg = Path()
        self.new_iteration = True
        self.x = 0.0
        self.y = 0.0
        self.tx = 0.0
        self.ty = 0.0
        self.start_x = self.x
        self.start_y = self.y
        self.last_x = self.x
        self.last_y = self.y
        self.delta = 0.005

    def goal_callback(self, msg):
        self.tx = msg.pose.position.x
        self.ty = msg.pose.position.y

        if self.x - self.delta < self.tx < self.x + self.delta or self.y - self.delta < self.ty < self.y + self.delta:
            self.last_x = self.x
            self.last_y = self.y
            self.new_iteration = False
        elif self.last_x != self.x or self.last_y != self.y:
            self.path_msg.poses = []
            self.new_iteration = True
        else:
            pass

    def location_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if self.new_iteration == True:
            self.path_msg.poses = []
            self.start_x = self.x
            self.start_y = self.y
            
            # print("Start point: ", self.start_x, self.start_y)
            # print("Target point: ", self.tx, self.ty)
            self.new_iteration = False

        # Draw path of the robot in RViz
        robot_pose = PoseStamped()
        robot_pose.pose.position.x = self.x
        robot_pose.pose.position.y = self.y
        self.path_msg.header.frame_id = 'odom'  # Set frame ID
        self.path_msg.poses.append(robot_pose)
        self.path_publisher.publish(self.path_msg)

    def publish_markers(self):
        marker_array = MarkerArray()

        # Create two markers representing the endpoints of the line
        point1 = Point()
        point1.x = self.start_x
        point1.y = self.start_y
        point1.z = 0.0

        point2 = Point()
        point2.x = self.tx
        point2.y = self.ty
        point2.z = 0.0

        marker = self.create_marker(point1, point2)
        marker1 = self.create_marker_points(point1)
        marker2 = self.create_marker_points(point2)

        # Set the same namespace/id for both markers to connect them
        marker.ns = "connected_markers"
        marker.id = 0
        marker1.ns = "connected_markers"
        marker1.id = 1
        marker2.ns = "connected_markers"
        marker2.id = 2

        # Add markers to the array
        marker_array.markers.append(marker)
        marker_array.markers.append(marker1)
        marker_array.markers.append(marker2)

        # Publish the MarkerArray
        self.marker_publisher.publish(marker_array)

    def create_marker(self, point1, point2):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.points = [point1, point2]
        marker.scale.x = 0.015 # Line width
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        return marker

    def create_marker_points(self, point):
        marker_point = Marker()
        marker_point.header.frame_id = "odom"
        marker_point.type = Marker.SPHERE
        marker_point.action = Marker.ADD
        marker_point.pose.position = point
        marker_point.pose.orientation.w = 1.0
        marker_point.scale.x = 0.075
        marker_point.scale.y = 0.075
        marker_point.scale.z = 0.075
        marker_point.color.a = 1.0
        marker_point.color.r = 0.0
        marker_point.color.g = 0.0
        marker_point.color.b = 1.0
        return marker_point

def main(args=None):

    rclpy.init(args=args)
    tracker_node = Tracker()

    try:
        while rclpy.ok():
            rclpy.spin(tracker_node)
    except KeyboardInterrupt:
        print("\nExiting program.\n")
    finally:
        tracker_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()