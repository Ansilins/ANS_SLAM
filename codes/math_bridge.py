import rclpy
import math
import struct
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

def main():
    rclpy.init()
    node = rclpy.create_node('math_bridge')
    
    # Hardcoding the exact QoS profile that made your RViz turn green
    qos_profile = QoSProfile(
        depth=10,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE
    )
    
    pub = node.create_publisher(PointCloud2, '/pointcloud', qos_profile)

    def callback(msg):
        data = bytearray()
        points_count = 0
        
        # Parse and filter points first to get an accurate count
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r) or r < msg.range_min or r > msg.range_max:
                continue
            
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            z = 0.0
            
            # Pack as 3 float32 values (4 bytes each = 12 bytes total per point)
            data.extend(struct.pack('fff', x, y, z))
            points_count += 1

        # Do not publish empty clouds
        if points_count == 0:
            return

        pc2 = PointCloud2()
        pc2.header = msg.header
        pc2.height = 1
        pc2.width = points_count
        pc2.fields = [
            PointField(name='x', offset=0, datatype=7, count=1), # 7 = FLOAT32
            PointField(name='y', offset=4, datatype=7, count=1),
            PointField(name='z', offset=8, datatype=7, count=1)
        ]
        pc2.is_bigendian = False
        pc2.point_step = 12
        pc2.row_step = pc2.point_step * points_count
        pc2.is_dense = False
        pc2.data = bytes(data)
        
        pub.publish(pc2)

    node.create_subscription(LaserScan, '/scan', callback, 10)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
