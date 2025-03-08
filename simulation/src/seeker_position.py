import os
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.publisher import Publisher

class SeekerPositionNode(Node):
    def __init__(self):
        super().__init__('seeker_position_node')
        
        self.publisher_ = self.create_publisher(Point, 'seeker_position', 10)
        
        self.sdf_file_path = os.path.join(os.getcwd(), 'world', 'easy.sdf')
        self.soldier_position = self.get_soldier_position()
        seeker_position = Point()
        seeker_position.x = self.soldier_position[0]
        seeker_position.y = self.soldier_position[1]
        seeker_position.z = 2.0  # Set z to 2 meters
        self.publisher_.publish(seeker_position)
        self.get_logger().info(f'Seeker position: {seeker_position}')

    def get_soldier_position(self):
        """Parse the easy.sdf file and extract the soldier's position (x, y)."""
        tree = ET.parse(self.sdf_file_path)
        root = tree.getroot()
        soldier_model = root.find(".//model[@name='soldier']")
        pose = soldier_model.find("pose").text.strip().split()
        x, y = float(pose[0]), float(pose[1])
        return x, y

def main(args=None):
    rclpy.init(args=args)
    
    seeker_position_node = SeekerPositionNode()
    
    rclpy.spin(seeker_position_node)
    
    seeker_position_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
