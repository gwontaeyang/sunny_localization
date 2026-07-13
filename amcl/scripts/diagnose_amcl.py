#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
from tf2_ros import TransformException

class AMCLDiagnostic(Node):
    def __init__(self):
        super().__init__('amcl_diagnostic')
        
        self.scan_received = False
        self.odom_received = False
        self.map_received = False
        self.amcl_pose_received = False
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_laserscan',
            self.scan_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(2.0, self.diagnose)
        
        self.get_logger().info('AMCL 진단 노드 시작')
    
    def scan_callback(self, msg):
        if not self.scan_received:
            self.scan_received = True
            self.get_logger().info('✅ /scan_laserscan 토픽 수신됨')
    
    def odom_callback(self, msg):
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('✅ /odom 토픽 수신됨')
    
    def map_callback(self, msg):
        if not self.map_received:
            self.map_received = True
            self.get_logger().info('✅ /map 토픽 수신됨')
    
    def amcl_pose_callback(self, msg):
        if not self.amcl_pose_received:
            self.amcl_pose_received = True
            self.get_logger().info('✅ /amcl_pose 토픽 수신됨')
    
    def diagnose(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('AMCL 진단 결과:')
        self.get_logger().info('=' * 60)
        
        if self.scan_received:
            self.get_logger().info('✅ /scan_laserscan: 수신 중')
        else:
            self.get_logger().warn('❌ /scan_laserscan: 수신 안 됨')
        
        if self.odom_received:
            self.get_logger().info('✅ /odom: 수신 중')
        else:
            self.get_logger().warn('❌ /odom: 수신 안 됨')
        
        if self.map_received:
            self.get_logger().info('✅ /map: 수신 중')
        else:
            self.get_logger().warn('❌ /map: 수신 안 됨')
        
        if self.amcl_pose_received:
            self.get_logger().info('✅ /amcl_pose: 수신 중')
        else:
            self.get_logger().warn('❌ /amcl_pose: 수신 안 됨')
        
        # TF 체인 확인
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            self.get_logger().info('✅ TF 체인: map -> base_link 존재')
        except TransformException as ex:
            self.get_logger().warn(f'❌ TF 체인: map -> base_link 실패 ({ex})')
        
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time()
            )
            self.get_logger().info('✅ TF 체인: odom -> base_link 존재')
        except TransformException as ex:
            self.get_logger().warn(f'❌ TF 체인: odom -> base_link 실패 ({ex})')
        
        self.get_logger().info('=' * 60)

def main(args=None):
    rclpy.init(args=args)
    node = AMCLDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
