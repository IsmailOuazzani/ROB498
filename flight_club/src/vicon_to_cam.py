#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

class PoseTransformNode(Node):
    def __init__(self):
        super().__init__('pose_transform_node')
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # Subscribers
        self.vicon_sub = self.create_subscription(
            PoseStamped,
            '/vicon/ROB498_Drone/ROB498_Drone',
            self.vicon_callback,
            10
        )
        self.camera_sub = self.create_subscription(
            Odometry,
            '/camera/pose/sample',
            self.camera_callback,
            qos_profile
        )
        
        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        self.stop_calibration_srv = self.create_service(Trigger, 'stop_calibration', self.stop_calibration)

        self.vicon_pose = None
        self.camera_pose = None
        self.offsets = []
        self.rotation_offsets = []
        self.offset = None
        self.logging_active = False
        self.last_sample_time = time.time()
        self.sample_interval = 1.0 / 5  # 5 samples per second
        self.get_logger().info("Vicon to camera running...")

    def vicon_callback(self, msg):
        # self.get_logger().info("Received vicon...")
        self.vicon_pose = msg
        if self.offset is None:
            print(msg)
            self.pose_pub.publish(msg)  # Initially publish raw Vicon data
        elif self.logging_active:
            self.log_error()

    def camera_callback(self, msg):
        # self.get_logger().info("Received camera...")
        current_time = time.time()
        if current_time - self.last_sample_time < self.sample_interval:
            return  # Skip this sample if not enough time has passed
        self.last_sample_time = current_time
        print(msg) 
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.camera_pose = pose
        
        if self.vicon_pose is not None and len(self.offsets) < 100:
            self.accumulate_offset()
        elif self.offset is not None:
            transformed_pose = self.apply_offset(pose)
            self.pose_pub.publish(transformed_pose)  # Publish transformed camera pose
    
    def stop_calibration(self, request, response):
        if not self.offsets:
            response.success = False
            response.message = "No offsets accumulated. Ensure both topics are publishing."
            return response
        
        
        self.offset = np.mean(self.offsets, axis=0)
        self.rotation_offset = R.from_quat(np.mean(self.rotation_offsets, axis=0))
        self.logging_active = True
        response.success = True
        response.message = "Calibration complete. Now publishing transformed camera poses."
        return response
    
    def accumulate_offset(self):
        self.get_logger().info(f"Save xform {len(self.offsets)}...")
        vicon_pos = np.array([
            self.vicon_pose.pose.position.x,
            self.vicon_pose.pose.position.y,
            self.vicon_pose.pose.position.z
        ])
        camera_pos = np.array([
            self.camera_pose.pose.position.x,
            self.camera_pose.pose.position.y,
            self.camera_pose.pose.position.z
        ])
        
        self.offsets.append(vicon_pos - camera_pos)
        self.get_logger().info(f"New offset: {vicon_pos - camera_pos}")
 
        vicon_quat = [
            self.vicon_pose.pose.orientation.x,
            self.vicon_pose.pose.orientation.y,
            self.vicon_pose.pose.orientation.z,
            self.vicon_pose.pose.orientation.w
        ]
        camera_quat = [
            self.camera_pose.pose.orientation.x,
            self.camera_pose.pose.orientation.y,
            self.camera_pose.pose.orientation.z,
            self.camera_pose.pose.orientation.w
        ]
        
        vicon_rot = R.from_quat(vicon_quat)
        camera_rot = R.from_quat(camera_quat)
        self.rotation_offsets.append((vicon_rot * camera_rot.inv()).as_quat())
    
    def stop_calibration(self, request, response):
        if not self.offsets:
            response.success = False
            response.message = "No offsets accumulated. Ensure both topics are publishing."
            return response
        
        self.offset = np.mean(self.offsets, axis=0)
        self.rotation_offset = R.from_quat(np.mean(self.rotation_offsets, axis=0))
        self.logging_active = True
        response.success = True
        response.message = "Calibration complete. Now publishing transformed camera poses."
        
        self.get_logger().info("Calibration stopped. Now transforming and publishing camera poses.")
        
        return response

    def apply_offset(self, pose):
        self.get_logger().info(f"applying offset {self.offset}")

        transformed_pose = PoseStamped()
        transformed_pose.header = pose.header
        
        pos = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
        print(pos)
        pos += self.offset
        print(pos)
        print("---")
        transformed_pose.pose.position.x = pos[0]
        transformed_pose.pose.position.y = pos[1]
        transformed_pose.pose.position.z = pos[2]
        
        quat = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ]
        
        #transformed_rot = self.rotation_offset * R.from_quat(quat)
        transformed_quat = quat # transformed_rot.as_quat()
        
        transformed_pose.pose.orientation.x = transformed_quat[0]
        transformed_pose.pose.orientation.y = transformed_quat[1]
        transformed_pose.pose.orientation.z = transformed_quat[2]
        transformed_pose.pose.orientation.w = transformed_quat[3]
        
        print(transformed_pose)
        return transformed_pose

    def log_error(self):
        if self.vicon_pose is None or self.camera_pose is None:
            return
        
        transformed_pose = self.apply_offset(self.camera_pose)
        
        error_pos = np.array([
            self.vicon_pose.pose.position.x - transformed_pose.pose.position.x,
            self.vicon_pose.pose.position.y - transformed_pose.pose.position.y,
            self.vicon_pose.pose.position.z - transformed_pose.pose.position.z
        ])
        
        #vicon_rot = R.from_quat([
        #    self.vicon_pose.pose.orientation.x,
        #    self.vicon_pose.pose.orientation.y,
        #    self.vicon_pose.pose.orientation.z,
        #    self.vicon_pose.pose.orientation.w
        #])
        #transformed_rot = R.from_quat([
        #    transformed_pose.pose.orientation.x,
        #    transformed_pose.pose.orientation.y,
        #    transformed_pose.pose.orientation.z,
        #    transformed_pose.pose.orientation.w
        #])
        
        #error_rpy = (vicon_rot.inv() * transformed_rot).as_euler('xyz', degrees=True)
        
        self.get_logger().info(f"Pose Error - X: {error_pos[0]:.4f}, Y: {error_pos[1]:.4f}, Z: {error_pos[2]:.4f}, ")
        #                       f"Roll: {error_rpy[0]:.2f}, Pitch: {error_rpy[1]:.2f}, Yaw: {error_rpy[2]:.2f}")
        

def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
