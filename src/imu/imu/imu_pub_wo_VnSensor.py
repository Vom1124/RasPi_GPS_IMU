#!/usr/bin/python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion, Vector3

from math import pow, radians

from imu.vectornav import Vectornav
import time
from datetime import datetime

# diag = radians(pow(.01, 2))
# cov_matrix = [diag, 0., 0., 0., diag, 0., 0., 0., diag]
position_cov_matrix = [-1.]*9
magnetic_cov_matrix = [0.]*9
# print(cov_matrix)

def setup_sensor(device = "/dev/ttyUSB0", baudrate = 115200, t0=datetime.now()):
    imu_sensor = Vectornav(1,device,baudrate,t0)
    imu_sensor.start
    return imu_sensor

def create_quaternion(data):
    obj = Quaternion()
    obj.x = data[0]
    obj.y = data[1]
    obj.z = data[2]
    obj.w = float(1)
    return obj

def create_vector3(data):
    obj = Vector3()
    obj.x = data[0]
    obj.y = data[1]
    obj.z = data[2] 
    return obj

def create_mat3(data):
    obj = []
    obj.append(data.e00)
    obj.append(data.e01)
    obj.append(data.e02)
    obj.append(data.e10)
    obj.append(data.e11)
    obj.append(data.e12)
    obj.append(data.e20)
    obj.append(data.e21)
    obj.append(data.e22)
    return obj

class ImuPublisher(Node):
    def __init__(self, imu_sensor = setup_sensor()):
        super().__init__('imu_publisher')

        self.sensor = imu_sensor

        self.frame_id = 'imu'

        self.publisher_position = self.create_publisher(Imu, f'{self.frame_id}/data', 10)
        self.publisher_magnetic = self.create_publisher(MagneticField, f'{self.frame_id}/mag', 10)
 
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # self.get_logger().info("Publishing imu data")
        imu_data = self.sensor.output_data()
        position_msg = Imu()

        # # Orientation
        orientation_reading = imu_data.ypr.T
        position_msg.orientation = create_quaternion(orientation_reading) 
        print(position_msg.orientation)

        # # Orientation covariance
        position_msg.orientation_covariance = position_cov_matrix
    
        # # Angular velocity
        angular_velocity_reading = imu_data.a
        position_msg.angular_velocity = create_vector3(angular_velocity_reading)

        # # Angular velocity covariance
        position_msg.angular_velocity_covariance = position_cov_matrix

        # # Linear acceleation
        #linear_acceleration_reading = self.sensor.read_acceleration_measurements()
        #position_msg.linear_acceleration = create_vector3(linear_acceleration_reading)

        # # Linear acceleation covariance
        # linear_acceleration_cov_reading = self.sensor.read_acceleration_compensation().c
        # position_msg.linear_acceleration_covariance = create_mat3(linear_acceleration_cov_reading)
        #position_msg.linear_acceleration_covariance = position_cov_matrix

        # # Magnetic Msg
        magnetic_msg = MagneticField()

        # # Read magnetic field
        magnetic_field_reading = imu_data.W
        print(magnetic_field_reading)
        magnetic_msg.magnetic_field = create_vector3(magnetic_field_reading)

        # # Read magnetic covariance
        # magnetic_field_cov_reading = self.sensor.read_magnetometer_compensation().c
        # magnetic_msg.magnetic_field_covariance = create_mat3(magnetic_field_cov_reading)
        magnetic_msg.magnetic_field_covariance = magnetic_cov_matrix

        # Msg info
        time = self.get_clock().now().to_msg()
        position_msg.header.stamp.sec = time.sec
        position_msg.header.stamp.nanosec = time.nanosec
        magnetic_msg.header.stamp.sec = time.sec
        magnetic_msg.header.stamp.nanosec = time.nanosec
        position_msg.header.frame_id = self.frame_id
        magnetic_msg.header.frame_id = self.frame_id

        # Publish position and magnetic
        self.publisher_position.publish(position_msg)
        self.publisher_magnetic.publish(magnetic_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
