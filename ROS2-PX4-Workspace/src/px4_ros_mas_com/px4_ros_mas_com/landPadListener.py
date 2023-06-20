import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
import re

class LandPadListener(Node):

    def __init__(self, landpad_station_num=1, landpad_num=1):
        super().__init__('land_pad_listener')

        self.drone_num_occupying=0

        self.landpad_station_num = landpad_station_num
        self.landpad_num = landpad_num

        self.occupied = False

        self.ver_drone_50=0

        self.drone_name = "iris"

        self.land_pad_contact_listener_ = self.create_subscription(String, f"/landpad_station_{landpad_station_num}/land_pad_{landpad_num}/contacts", self.listener_callback, 10)


    def listener_callback(self, msg):
        if self.ver_drone_50 > 50:
            self.ver_drone_50 = 0
            if self.occupied:
                self.get_logger().info(f'Land Pad {self.landpad_num} on Station {self.landpad_station_num} AVAILABLE')
                self.occupied = False
        self.drone_num_occupying = self.validate_string(msg.data)
        if self.drone_num_occupying > 0:
            if not self.occupied:
                self.get_logger().info(f'Drone {self.drone_num_occupying} Attached to Land Pad {self.landpad_num} on Station {self.landpad_station_num}')
                self.occupied = True
                self.ver_drone_50 = 0
        else:
            self.ver_drone_50+=1
    
    def validate_string(self, input_string):
        match = re.search(r'iris_(\d+)', input_string)
        if match:
            return int(match.group(1))
        else:
            return 0
        