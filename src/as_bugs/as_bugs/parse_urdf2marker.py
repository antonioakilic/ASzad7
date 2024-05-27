import os
import xacro
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker,MarkerArray
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET

class ObstacleCoursePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_course_publisher')
        
        # Load URDF file and process it with xacro
        pkg_name = 'as_bugs'
        urdf_file = os.path.join(get_package_share_directory(pkg_name), 'models', 'arena.urdf.xacro')
        self.urdf_xml = xacro.process_file(urdf_file).toxml()
        
        # Parse URDF and extract obstacle geometry information
        self.objects = self.parse_urdf(self.urdf_xml)
        
        # Publish markers
        self.marker_publisher = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)
        
    def parse_urdf(self, urdf_xml):
        tree = ET.ElementTree(ET.fromstring(urdf_xml))
        root = tree.getroot()

        objects = {}

        for link in root.findall('link'):
            link_name = link.attrib.get('name')
            if link_name is None:
                continue

            visual = link.find('visual')
            if visual is not None:
                
                origin = visual.find('origin')
                if origin is not None:
                    origin_xyz = origin.attrib.get('xyz', '0 0 0')
                    origin_rpy = origin.attrib.get('rpy', '0 0 0')
                    xyz = [float(x) for x in origin_xyz.split()]
                    rpy = [float(x) for x in origin_rpy.split()]
                else:
                    xyz = [0.0, 0.0, 0.0]
                    rpy = [0.0, 0.0, 0.0]
                    
                geometry = visual.find('geometry')
                if geometry is not None:
                    box = geometry.find('box')
                    if box is not None:
                        size_str = box.attrib.get('size')
                        if size_str:
                            size = [float(x) for x in size_str.split()]
                        else:
                            size = [0.0, 0.0, 0.0]
                            
                            
                        # print("Link: ", link_name, " XYZ:", xyz, " RPY: ", rpy)
                        objects[link_name] = {'size': size, 'xyz': xyz, 'rpy': rpy}

        for joint in root.findall('joint'):
            joint_name = joint.attrib.get('name')
            if joint_name is None:
                continue

            parent_link = joint.find('parent').attrib.get('link')
            child_link = joint.find('child').attrib.get('link')
            origin = joint.find('origin')
            if origin is not None:
                origin_xyz = origin.attrib.get('xyz', '0 0 0')
                origin_rpy = origin.attrib.get('rpy', '0 0 0')
                xyz = [float(x) for x in origin_xyz.split()]
                rpy = [float(x) for x in origin_rpy.split()]
            else:
                xyz = [0.0, 0.0, 0.0]
                rpy = [0.0, 0.0, 0.0]

            objects[child_link]['parent'] = parent_link
            objects[child_link]['joint_pose'] = {'xyz': xyz, 'rpy': rpy}
            
        return objects
        
    def publish_markers(self):
        marker_array = MarkerArray()
        marker_id = 0  # Initialize marker ID counter

        for obj_name, obj_info in self.objects.items():
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.type = Marker.CUBE  # Change type as needed
            marker.action = Marker.ADD

            if 'joint_pose' in obj_info:
                joint_pose = obj_info['joint_pose']
                obj_xyz = obj_info['xyz']
                obj_rpy = obj_info['rpy']

                # Transform object pose based on joint pose
                rotated_xyz = np.array(obj_xyz)
                rotated_rpy = np.array(obj_rpy)
                for axis, value in enumerate(joint_pose['xyz']):
                    rotated_xyz[axis] += value
                for axis, value in enumerate(joint_pose['rpy']):
                    rotated_rpy[axis] += value

                qx = np.sin(rotated_rpy[0] / 2) * np.cos(rotated_rpy[1] / 2) * np.cos(rotated_rpy[2] / 2) - \
                    np.cos(rotated_rpy[0] / 2) * np.sin(rotated_rpy[1] / 2) * np.sin(rotated_rpy[2] / 2)
                qy = np.cos(rotated_rpy[0] / 2) * np.sin(rotated_rpy[1] / 2) * np.cos(rotated_rpy[2] / 2) + \
                    np.sin(rotated_rpy[0] / 2) * np.cos(rotated_rpy[1] / 2) * np.sin(rotated_rpy[2] / 2)
                qz = np.cos(rotated_rpy[0] / 2) * np.cos(rotated_rpy[1] / 2) * np.sin(rotated_rpy[2] / 2) - \
                    np.sin(rotated_rpy[0] / 2) * np.sin(rotated_rpy[1] / 2) * np.cos(rotated_rpy[2] / 2)
                qw = np.cos(rotated_rpy[0] / 2) * np.cos(rotated_rpy[1] / 2) * np.cos(rotated_rpy[2] / 2) + \
                    np.sin(rotated_rpy[0] / 2) * np.sin(rotated_rpy[1] / 2) * np.sin(rotated_rpy[2] / 2)

                marker.pose = Pose(position=Point(x=rotated_xyz[0], y=rotated_xyz[1], z=rotated_xyz[2]),
                                    orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))
            else:
                marker.pose = Pose(position=Point(x=obj_info['xyz'][0], y=obj_info['xyz'][1], z=obj_info['xyz'][2]),
                                    orientation=Quaternion(x=0, y=0, z=0, w=1))

            marker.scale.x = obj_info['size'][0]
            marker.scale.y = obj_info['size'][1]
            marker.scale.z = obj_info['size'][2]
            marker.color.a = 0.5  # Transparency
            marker.color.r = 1.0 if 'box' in obj_name else 0.4
            marker.color.g = 0.0 if 'box' in obj_name else 0.2
            marker.color.b = 0.0

            # Assign unique namespace (ns) and ID for each marker
            marker.ns = str(obj_name)  # Use object name as namespace
            marker.id = marker_id  # Use unique ID
            marker_id += 1  # Increment marker ID counter

            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)
    
    
def main():
    rclpy.init()
    obstacle_course_publisher = ObstacleCoursePublisher()
    rclpy.spin(obstacle_course_publisher)
    obstacle_course_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()