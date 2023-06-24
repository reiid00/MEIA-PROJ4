import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re


class ChargingSpotListener(Node):

    def __init__(self, charging_station_num=1, charging_spot_num=1, x=0.0, y=0.0):
        super().__init__('charging_spot_listener')
        self.drone_num_occupying=0
        self.charging_station_num = charging_station_num
        self.charging_spot_num = charging_spot_num
        self.occupied = False
        self.ver_drone_50=0
        self.x, self.y = x, y
        self.charging_spot_contact_listener_ = self.create_subscription(String, f"/charging_station_{charging_station_num}/charging_spot_{charging_spot_num}/contacts", self.listener_callback, 10)
        self.drone_assigned = False

    def get_type(self):
        return "CHARGING_STATION"
    
    def get_station(self):
        return f"Charging Spot {self.charging_spot_num} on Station {self.charging_station_num}"
    
    def get_location(self):
        return self.x, self.y
    
    def is_available(self):
        if self.occupied or self.drone_assigned:
            return False
        return True
    
    def assign_drone(self):
        self.drone_assigned = True
    
    def listener_callback(self, msg):
        if self.ver_drone_50 > 50:
            self.ver_drone_50 = 0
            if not self.is_available():
                self.get_logger().info(f'Charging Spot {self.charging_spot_num} on Station {self.charging_station_num} AVAILABLE')
                self.occupied = False
                self.drone_assigned = False
        if not self.occupied:
            self.drone_num_occupying = self.validate_string(msg.data)
            if self.drone_num_occupying > 0:
                    self.get_logger().info(f'Drone {self.drone_num_occupying} Attached to Charging Spot {self.charging_spot_num} on Station {self.charging_station_num}')
                    self.occupied = True
                    self.ver_drone_50 = 0
            elif not self.drone_assigned:
                self.ver_drone_50+=1
        
    def validate_string(self, input_string):
        match = re.search(r'iris_(\d+)', input_string)
        return int(match.group(1)) if match else 0