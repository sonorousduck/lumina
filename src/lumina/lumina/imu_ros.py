import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, Point, Pose
import tf_transformations as tf_trans  # For quaternion conversions
from .berryIMU import BerryIMU

import numpy as np

# i2c interface

class IMU_ROS(Node):
    def __init__(self):
        super().__init__("imu")

        self.imu_publisher = self.create_publisher(Imu, "/imu/data", 10)
        self.magnetometer_publisher = self.create_publisher(MagneticField, "/imu/compass", 10)
        self.position_publisher = self.create_publisher(PoseStamped, "/imu/pose", 10)

        self.timer_period = 1  # Seconds (1 / 200 Hz)
        self.timer = self.create_timer(self.timer_period, self.publish)

        self.imu = BerryIMU()


    def publish(self):
        self.imu.read_imu()

        imu_msg = Imu()
        magnetometer_msg = MagneticField()
        position_msg = PoseStamped()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        magnetometer_msg.header.stamp = imu_msg.header.stamp
        position_msg.header.stamp = imu_msg.header.stamp
        imu_msg.header.frame_id = "imu_link"
        magnetometer_msg.header.frame_id = "imu_link"
        position_msg.header.frame_id = "imu_link"

        # Convert Kalman angles to quaternion
        quaternion = tf_trans.quaternion_from_euler(self.imu.roll, self.imu.pitch, 0.0)
        imu_msg.orientation = Quaternion()
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]


        # Angular velocity
        imu_msg.angular_velocity.x = self.imu.gyroXangle
        imu_msg.angular_velocity.y = self.imu.gyroYangle
        imu_msg.angular_velocity.z = self.imu.gyroZangle

        # Linear Acceleration
        imu_msg.linear_acceleration.x = self.imu.accXnorm
        imu_msg.linear_acceleration.y = self.imu.accYnorm

        self.imu_publisher.publish(imu_msg)

        # Magnetometer
        magnetometer_msg.magnetic_field = Vector3(x=self.imu.magXcomp, y=self.imu.magYcomp, z=np.NaN)

        self.magnetometer_publisher.publish(magnetometer_msg)

        # Position
        position_msg.pose = Pose()
        position_msg.pose.position = Point(x=self.imu.kalmanX, y=self.imu.kalmanY)
        position_msg.pose.orientation = imu_msg.orientation

        self.position_publisher.publish(position_msg) 






def main(args=None):
    rclpy.init(args=args)

    imu = IMU_ROS()

    rclpy.spin(imu)

    imu.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()