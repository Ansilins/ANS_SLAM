#!/usr/bin/env python3
import os, time, math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# --- HARDWARE SETUP ---
PWM_DIR = "/sys/class/pwm/pwmchip0/pwm0"
DUTY_PATH = f"{PWM_DIR}/duty_cycle"

# Locked Calibration
ZERO_NS   = 1700000
CCW_20_NS = 1477780   # -45°
CW_20_NS  = 1922220   # +45°

# Initialize Sysfs PWM
if not os.path.exists(PWM_DIR):
    with open("/sys/class/pwm/pwmchip0/export", "w") as f: f.write("0")
    time.sleep(1)
with open(f"{PWM_DIR}/period", "w") as f: f.write("20000000")
with open(f"{PWM_DIR}/enable", "w") as f: f.write("1")

def set_duty(ns):
    with open(DUTY_PATH, "w") as f: f.write(str(int(ns)))

# --- THE ROS 2 FUSION NODE ---
class ServoHardwareBridge(Node):
    def __init__(self, speed_rad_sec=1.885):
        super().__init__('servo_hardware_bridge')
        self.br = TransformBroadcaster(self)

        # Restored your original speed calculation
        self.speed = speed_rad_sec / (1 * math.pi)
        self.start_time = time.time()

        # Run at 100Hz
        self.timer = self.create_timer(0.01, self.sync_callback)
        self.get_logger().info("Hardware & TF Sync Active: Sweeping ±45° at NORMAL SPEED")

    def sync_callback(self):
        elapsed = time.time() - self.start_time
        wave = math.sin(2 * math.pi * self.speed * elapsed)

        # 1. DRIVE THE PHYSICAL SERVO
        if wave >= 0:
            ns = ZERO_NS + wave * (CW_20_NS  - ZERO_NS)
        else:
            ns = ZERO_NS + wave * (ZERO_NS   - CCW_20_NS)
        set_duty(ns)

        # 2. UPDATE THE DIGITAL TWIN (ROS 2 TF)
        pitch = -wave * (math.pi / 9.0)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1 

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = math.sin(pitch / 2.0)
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = math.cos(pitch / 2.0)

        self.br.sendTransform(t)

def main():
    try:
        print("Parking at 0°...")
        set_duty(ZERO_NS)
        time.sleep(1)

        rclpy.init()
        # Restored explicitly to 1.885 rad/s
        node = ServoHardwareBridge(speed_rad_sec=3.141)
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\nShutting down. Parking at 0°...")
    finally:
        set_duty(ZERO_NS)
        time.sleep(0.5)
        with open(f"{PWM_DIR}/enable", "w") as f: f.write("0")
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
