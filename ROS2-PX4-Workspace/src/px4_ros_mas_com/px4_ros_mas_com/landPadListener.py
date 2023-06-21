import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
import re

class LandPadListener(Node):

    def __init__(self, landpad_station_num=1, landpad_num=1, x=0.0, y=0.0):
        super().__init__('land_pad_listener')
        self.drone_num_occupying=0
        self.landpad_station_num = landpad_station_num
        self.landpad = landpad_num
        self.occupied = False
        self.x, self.y = x, y
        self.ver_drone_50=0
        self.drone_name = "iris"
        self.land_pad_contact_listener_ = self.create_subscription(String, f"/landpad_station_{landpad_station_num}/land_pad_{landpad_num}/contacts", self.listener_callback, 10)
        self.drone_assigned = False
    
    def get_type(self):
        return "LAND_PAD"
    
    def get_station(self):
        return f"Land Pad {self.landpad} on Station {self.landpad_station_num}"
    
    def get_location(self):
        return self.x, self.y
    
    def is_available(self):
        if self.occupied or self.drone_assigned:
            return False
        return True
    
    def assign_drone(self):
        self.drone_assigned = True
        self.ver_drone_50 = 0
    
    def listener_callback(self, msg):
        if self.ver_drone_50 > 50:
            self.ver_drone_50 = 0
            if not self.is_available():
                self.get_logger().info(f'Land Pad {self.landpad} on Station {self.landpad_station_num} AVAILABLE')
                self.occupied = False
                self.drone_assigned = False
        if not self.occupied:
            self.drone_num_occupying = self.validate_string(msg.data)
            if self.drone_num_occupying > 0:
                    self.get_logger().info(f'Drone {self.drone_num_occupying} Attached to Land Pad {self.landpad} on Station {self.landpad_station_num}')
                    self.occupied = True
                    self.ver_drone_50 = 0
            elif not self.drone_assigned:
                self.ver_drone_50+=1
    
    def validate_string(self, input_string):
        match = re.search(r'iris_(\d+)', input_string)
        return int(match.group(1)) if match else 0
        