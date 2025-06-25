import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import math
from transforms3d.euler import euler2quat

class ImuSerialReader(Node):
    def __init__(self):
        super().__init__('imu_serial_reader')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        try:
            self.ser = serial.Serial('/dev/ttyUSB1', 115200)  # Change to the correct serial port if necessary
            self.get_logger().info('Serial port opened successfully')
        except serial.SerialException as e:
            self.get_logger().error('Failed to open serial port: {}'.format(e))
            return
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            # self.get_logger().info(f'Received line: {line}')
            data = line.split(", ")

            if len(data) == 9:
                try:
                    accel_x = float(data[0].split(": ")[1])
                    accel_y = float(data[1].split(": ")[1])
                    accel_z = float(data[2].split(": ")[1])
                    gyro_x = float(data[3].split(": ")[1])
                    gyro_y = float(data[4].split(": ")[1])
                    gyro_z = float(data[5].split(": ")[1])
                    mag_x = float(data[6].split(": ")[1])
                    mag_y = float(data[7].split(": ")[1])
                    mag_z = float(data[8].split(": ")[1])



                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = 'base_link'
                    imu_msg.linear_acceleration.x = accel_x
                    imu_msg.linear_acceleration.y = accel_y
                    imu_msg.linear_acceleration.z = accel_z
                    imu_msg.angular_velocity.x = gyro_x
                    imu_msg.angular_velocity.y = gyro_y
                    imu_msg.angular_velocity.z = gyro_z

                    # Calculate orientation using a simple tilt-compensated compass algorithm
                    #roll = math.atan2(accel_y, accel_z)
                    #pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
                    #yaw = math.atan2(mag_y * math.cos(roll) - mag_z * math.sin(roll),
                    #                mag_x * math.cos(pitch) + mag_y * math.sin(pitch) * math.sin(roll) + mag_z * math.sin(pitch) * math.cos(roll))
                    
                    roll = 0
                    pitch = 0
                    yaw = math.atan2(mag_y, mag_x)
                    
                    # Convert yaw, pitch, roll to quaternion using transforms3d
                    quaternion = euler2quat(roll, pitch, yaw)
                    imu_msg.orientation.w = quaternion[0]
                    imu_msg.orientation.x = quaternion[1]
                    imu_msg.orientation.y = quaternion[2]
                    imu_msg.orientation.z = quaternion[3]

                    #self.get_logger().info(f' Acc: {accel_x}, {accel_y}, {accel_z}, Gyr: {gyro_x}, {gyro_y}, {gyro_z}, Mag: {mag_x}, {mag_y}, {mag_z}, w: {imu_msg.orientation.w:.4f}, x: {imu_msg.orientation.x:.4f}, y: {imu_msg.orientation.y:.4f}, z: {imu_msg.orientation.z:.4f}')
                    self.get_logger().info(f'Gyr: {gyro_x}, {gyro_y}, {gyro_z},   Mag: {mag_x}, {mag_y}, {mag_z}')

                    #self.get_logger().info(f' Mag: {mag_x}, {mag_y}, {mag_z}')

                    # Adjusted covariances based on Gazebo simulated data 0.000289   4.0e-08
                    imu_msg.linear_acceleration_covariance = [0.000289, 0.0, 0.0,
                                                              0.0, 0.000289, 0.0,
                                                              0.0, 0.0, 0.000289]
                    imu_msg.angular_velocity_covariance = [4.0e-08, 0.0, 0.0,
                                                           0.0, 4.0e-08, 0.0,
                                                           0.0, 0.0, 4.0e-08]
                    imu_msg.orientation_covariance = [-1.0, 0.0, 0.0,
                                                      0.0, -1.0, 0.0,
                                                      0.0, 0.0, -1.0,]  # -1 indicates unknown

                    self.publisher_.publish(imu_msg)
                    #self.get_logger().info('Published IMU data')

                except ValueError as e:
                    self.get_logger().error(f'Error parsing data: {e}')
            else:
                self.get_logger().error('Received line does not contain expected number of values')
        except serial.SerialException as e:
            self.get_logger().error(f'Error reading serial data: {e}')

def main(args=None):
    rclpy.init(args=args)
    imu_serial_reader = ImuSerialReader()
    rclpy.spin(imu_serial_reader)
    imu_serial_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
