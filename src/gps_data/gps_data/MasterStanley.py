from find_ports import search_port
import math
import numpy as np
import rclpy
import serial
import struct
import time
from std_msgs.msg import String
import pymap3d as pm
import csv
from rclpy.node import Node
from std_msgs.msg import String

global latitude
global longitude

latitude, longitude, altitude = 0, 0, 0
ENU_x, ENU_y, ENU_z = 0, 0, 0

#search_port("u-blox GNSS receiver")arduino_port = search_port("Arduino")

arduino = serial.Serial(port=search_port("Arduino"), baudrate=115200, timeout=0.1)

#data = 'gps_ENU.csv'
with open("/home/tera/ros2_ws2/src/gps_data/gps_data/gps_data_tera_fresh1.csv", "r") as csvfile:
    reader = csv.reader(csvfile, quotechar='|')
    next(reader)
    data = []
    for row in reader:
        x = float(row[0])
        y = float(row[1])
        z = float(row[2])
        data.append([x, y])
    print(f"Path length is {len(data)} points")

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'tera_teleop', 10)
        timer_period = 0.05 #seconds
        self.timer = self.create_timer(timer_period, self.input)
        print("Starting Stanley teleop")

    def input(self, data):
        msg = String()
        msg.data = (f"{data[0]},{data[1]}")
        self.publisher_.publish(msg)    
        print(f"Msg: ", msg)

class GPS_Subscriber(Node):
    def __init__(self):
        #rclpy.init(args=None)
        super().__init__('Stanley_subscriber')
        self.subscription = self.create_subscription(
            String,
            'gps_data_tera',
            self.listener_callback,
            10)
        self.subscription
        self.lat0, self.lon0, self.h0 = 58.3428685594, 25.5692475361, 91.357
        self.latitude, self.longitude, self.altitude = 0, 0, 0
        self.ENU_x, self.ENU_y = 0, 0
        
        

    def listener_callback(self, msg):
        #print("CALLBACK")
        self.get_logger().info('I heard "%s"' %msg.data)
        info = msg.data.split(',')
        self.latitude = float(info[0])
        self.longitude = float(info[1])
        self.altitude = float(info[2])
        self.ENU_x, self.ENU_y, ENU_z = pm.geodetic2enu(self.latitude, self.longitude, self.altitude, self.lat0, self.lon0, self.h0)
        
        
        #print("gps.listener_callback")

    def get_coordinates(self):
        #print("got coords")
        return self.latitude, self.longitude
        
    
    def get_ENU(self):
        return self.ENU_x, self.ENU_y

class StanleyController():
    def __init__(self):
        self.target_location = []
        self.target_x = 0
        self.target_y = 0
        self.last_x = 0
        self.last_y = 0
        #self.h_err_corr
        self.xt_err_corr
        self.k = 0.5
        self.Kp = 1.0
        self.dt = 0.1
        self.L = 1.0
        self.max_steer = np.deg2rad(45)
        self.steering_command = 0
        self.suber = GPS_Subscriber()
        
    def get_target_location(self):
        i = 0
        min_dist = 100000000
        for i in range(len(data)):
            #current_x, current_y = self.suber.get_gps_data()
            current_min_dist = math.dist(data[i], [self.latitude, self.longitude])

            if current_min_dist < min_dist:
                min_dist = current_min_dist
                min_dist_idx = i

        return data[min_dist_idx]

    def xt_err_corr(self, current_heading, k_s): # The cross-track error is the distance between the vehicle and the ideal traiectory

        v_p = np.array([np.sin(current_heading), -np.cos(current_heading)])
        v_t = np.array([np.cos(current_heading), np.sin(current_heading)])
        #print(f"v_p:", v_p)
        #print(f"v_t:",v_t)
        xt_error = np.linalg.norm(np.cross(v_p, v_t))
        xt_err_correction = np.arctan((xt_error * self.k) / (self.Kp + k_s))
        return xt_err_correction 

    def steering_control(self, next_heading, current_heading, target_position, current_position, k, v, k_s): # The steering control is the sum of the heading error correction and the cross-track error correction. 
        heading_err = next_heading - current_heading
        xt_err = self.xt_err_corr(current_heading, k_s)
        return heading_err + xt_err
    
    def get_gps_trace(self):
        return data


    def implementation(self):
        current_position = [self.ENU_x, self.ENU_y]
        #print(f"Current pos: ", current_position)
        nearest_point = self.get_target_location()
        #print(f"Nearest: ", nearest_point)
        #trace_length = len(self.get_gps_trace())
        #trace_length = len(data)
        #print(f"Trace length: ", trace_length)
        #data_array = self.get_gps_trace()
        #data_array = data
        #print("data_array done")
        #print(self.get_gps_trace())
        next_heading_x = nearest_point[0] - current_position[0]
        next_heading_y = nearest_point[1] - current_position[1]
        next_heading = np.arctan2(next_heading_y, next_heading_x)
        #for i in range(nearest_point[1], (trace_length - 1)):
            #next_heading = np.arctan2(data_array[i+1][0] - data_array[i][0], data_array[i+1][1] - data_array[i][1])
        current_heading = np.arctan2(current_position[1] - self.last_y, current_position[0] - self.last_x)
        #print(f"Next heading", next_heading)
        #print(f"Current heading", current_heading)
        target_position = self.get_target_location()
        #print(f"next target position", target_position)
        self.steering_command = np.rad2deg(np.clip(self.steering_control(next_heading, current_heading, target_position, current_position, self.k, 1.5, 1), -self.max_steer, self.max_steer))
        self.last_x = current_position[0]
        self.last_y = current_position[1]

        def map_range(x, in_min, in_max, out_min, out_max):
            return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
        
        self.steering_command = int(map_range(self.steering_command, -45, 45, 50, 600))

        if self.steering_command == None:
            return 0
        else:
            return int(self.steering_command)
            
    def send2arduino(self, lat, lon, x, y):
        self.latitude = lat
        self.longitude = lon
        self.ENU_x = x
        self.ENU_y = y
        steering = self.implementation()
        #print(type(steering))
        #print(f"steering value: ", steering)
        #print("\n")
        #to_send = [int(steering), int(100)]
        #print("TO send:", to_send)
        pub.input([steering, 125])
        
        time.sleep(0.2)
        '''
        sent = arduino.write(struct.pack("<2h", *[int(steering), int(100)]))
        print("To arduino: ", sent)
        time.sleep(0.5)
        try:
            (val, ) = struct.unpack("h", arduino.read(struct.calcsize("h")))
            print("Received: ", val)
        except:
            print("Got nothing from arduino")
            pass
        '''
        

if __name__ == '__main__':
    rclpy.init(args=None)
    gps = GPS_Subscriber()
    pub =MinimalPublisher()
    #TERA_gps.main()
    stan = StanleyController()
    while True:
        #print("tsüklis")
        rclpy.spin_once(gps)
        #print("gps init done")
        latitude, longitude = gps.get_coordinates()
        #print("got gps coords")
        x, y = gps.get_ENU()
        #print("got gps ENU")

        stan.send2arduino(latitude, longitude, x, y)