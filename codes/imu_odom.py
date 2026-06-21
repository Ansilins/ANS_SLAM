#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import time

import board
import busio
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C

class USVOdometryBroadcaster(Node):
    def __init__(self):
        super().__init__('usv_odometry_broadcaster')
        self.br = TransformBroadcaster(self)
        
        self.get_logger().info('Initializing BNO085 IMU for USV Odometry...')
        
        # --- WARMUP TIMER SETUP ---
        self.start_time = time.time()
        self.warmup_duration = 30.0  # 30 seconds of pure calibration
        self.warmup_complete = False
        
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
            # 0x4a is standard, 0x4b is the alternate if the address pin is pulled high
            self.bno = BNO08X_I2C(self.i2c, address=0x4a)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            
            # Read at 100Hz (0.01 seconds) to match your servo sweep frequency
            self.timer = self.create_timer(0.01, self.timer_callback)
            self.get_logger().info('IMU Connected! Starting 30-second hardware warmup to prevent drift...')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to IMU: {e}')

    def timer_callback(self):
        try:
            # Grab the live Quaternions directly from the BNO085 coprocessor to clear the buffer
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
            
            # --- WARMUP LOGIC ---
            elapsed_time = time.time() - self.start_time
            if elapsed_time < self.warmup_duration:
                # Print a status update every 5 seconds to the terminal
                if int(elapsed_time * 100) % 500 == 0:
                    remaining = int(self.warmup_duration - elapsed_time)
                    self.get_logger().info(f'Warming up... {remaining} seconds left before mapping starts.')
                return # Exit early! Do NOT broadcast to RTAB-Map yet.
            
            # Trigger once when warmup finishes
            if not self.warmup_complete:
                self.get_logger().info('Warmup complete! Broadcasting clean odom -> base_link to RTAB-Map.')
                self.warmup_complete = True
            
            # --- NORMAL OPERATION ---
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            
            # odom is the stationary water surface. base_link is the boat.
            t.header.frame_id = 'odom'       
            t.child_frame_id = 'base_link'   

            # Since we don't have GPS or SLAM running yet, the boat stays at 0,0,0
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0 

            # Apply the IMU rotation to the entire chassis
            t.transform.rotation.x = quat_i
            t.transform.rotation.y = quat_j
            t.transform.rotation.z = quat_k
            t.transform.rotation.w = quat_real

            self.br.sendTransform(t)
        except Exception as e:
            # Silently pass on occasional I2C clock-stretching dropped frames
            pass 

def main(args=None):
    rclpy.init(args=args)
    node = USVOdometryBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
