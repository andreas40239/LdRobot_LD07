# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Performance on Raspberry Pi4b with Ubuntu 20.04 LTS and ROS2 Galactic
# CPU: 15%, RAM 580M virt., 10Hz bandwith in RQT topic monitor

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import serial
import binascii
import math
import time



class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'LiDAR/LD07', 1)
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.ser = serial.Serial(port='/dev/ttyUSB0',    
                            baudrate=921600,
                            timeout=10.0,
                            bytesize=8,
                            parity='N',
                            stopbits=1)

        self.ld07_startMeasurement = [0xAA,0xAA,0xAA,0xAA,0x01,0x02,0x00,0x00,0x00,0x00,0x03]
        self.ld07_stopMeasurement = [0xAA,0xAA,0xAA,0xAA,0x01,0x0F,0x00,0x00,0x00,0x00,0x10]
        self.ld07_header = 0xAA
        self.ld07_angleIncrement = 1.5708 / 160 # 90 degrees FOV / 160 measure points
        self.ld07_distances = list()
        self.ld07_confidences = list()

        self.ser.write(self.ld07_startMeasurement)
        time.sleep(0.1)



    def timer_callback(self):

        while True:

            ### Search for header ################################
            searchHeader = True
            while searchHeader:
                
                #read data from interface until header 0xAA received
                headernotFound = True
                while headernotFound:
                    b = self.ser.read(1)
                    tmpInt = int.from_bytes(b, 'little')
                    if tmpInt == self.ld07_header:
                        #repeat 3 times
                        headernotFound = False

                # next 3 times must be 0xAA, too.
                # I am too distracted to code a beautiful algorithm now :-/
                #next byte must be 0xAA
                b = self.ser.read(1)
                tmpInt = int.from_bytes(b, 'little')
                if tmpInt == self.ld07_header:
                    #next byte must be 0xAA
                    b = self.ser.read(1)
                    tmpInt = int.from_bytes(b, 'little')
                    if tmpInt == self.ld07_header:
                        #next byte must be 0xAA
                        b = self.ser.read(1)
                        tmpInt = int.from_bytes(b, 'little')
                        if tmpInt == self.ld07_header:
                            searchHeader = False


            ### Read data frame ################################

            #reading first data ...
            ld07_deviceAddress = int.from_bytes(self.ser.read(1), byteorder='little')
            ld07_cmdCode = int.from_bytes(self.ser.read(1), byteorder='little')
            ld07_packetOffsetAddress = int.from_bytes(self.ser.read(2), byteorder='little')
            ld07_dataLength = int.from_bytes(self.ser.read(2), byteorder='little')
            ld07_timestamp = int.from_bytes(self.ser.read(4), byteorder='little')

            #reading all 160 distance measurement points
            for measurement in range(160):
                tmpMeasurement = int.from_bytes(self.ser.read(2), byteorder='little')
                tmpConfidence = (tmpMeasurement >> 9) <<1
                tmpDistance = tmpMeasurement & 0x1ff
                self.ld07_distances.append(float(tmpDistance/1000))
                self.ld07_confidences.append(float(tmpConfidence))


            ### Publish data frame ################################
            scanData = LaserScan()
            scanData.header.stamp = self.get_clock().now().to_msg()   
            scanData.header.frame_id = "laser_frame"
            scanData.angle_min = 5.49779 # 315 degree
            scanData.angle_max = 0.785398 # 45 degree
            scanData.angle_increment = self.ld07_angleIncrement 
            scanData.time_increment = 0.0002
            scanData.range_min = 0.005   # 5 mm
            scanData.range_max = 0.4    # 40 cm
            scanData.ranges = self.ld07_distances
            scanData.intensities = self.ld07_confidences
            self.publisher_.publish(scanData)
            
            self.ld07_distances.clear()
            self.ld07_confidences.clear()





def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
