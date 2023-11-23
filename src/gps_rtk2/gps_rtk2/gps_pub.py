#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node

from  serial import Serial 
import pynmea2 as nmea
from io import TextIOWrapper, BufferedRWPair

from sensor_msgs.msg import NavSatFix

from datetime import datetime
from datetime import timezone
import pytz

def setup_device(device='/dev/ttyACM0', baudrate=4800, timeout=1):

    os.system("echo 1124 | sudo -S chmod 0777 /dev/ttyACM0")
    serial = Serial(device, baudrate, timeout=timeout)    
    serial_data = TextIOWrapper(BufferedRWPair(serial, serial))
    return serial_data

class GPS(Node):


    def __init__(self, serial_connection=setup_device(), timer=0.1):
        super().__init__("gps_publisher")

        self.sw = serial_connection

        self.publisher = self.create_publisher(NavSatFix, 'NavSatFix', 10)

        self.timer = self.create_timer(timer, self.read)

    def read(self):
        try:
            data = self.sw.readline()
            if "$GNGLL" in data:
                msg = nmea.parse(data)
                default_time = msg.timestamp.replace(tzinfo=pytz.UTC)
                #local_time  
                # print(default_time)
                # print(msg)
                print(repr(msg))
                print("Altitude{}".format(msg.altitude))
                print("Latitude:{}".format(msg.lat))
                print("Longitude:{}".format(msg.lon))
                
                # GPS Msg
                gps_msg = NavSatFix()
                gps_msg.altitude = float(msg.altitude)
                gps_msg.latitude = float(msg.latitude)
                gps_msg.longitude = float(msg.longitude)
                
                self.publisher.publish(gps_msg)

        except Exception: pass

def main(args=None):
    rclpy.init(args=args)
    node = GPS()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
