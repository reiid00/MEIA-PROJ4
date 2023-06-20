import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
import re


class ChargingSpotListener(Node):

    def __init__(self, charging_station_num=1, charging_spot_num=1):
        super().__init__('charging_spot_listener')
        self.drone_num_occupying=0

        self.charging_station_num = charging_station_num
        self.charging_spot_num = charging_spot_num

        self.occupied = False

        self.ver_drone_50=0

        self.charging_spot_contact_listener_ = self.create_subscription(String, f"/charging_station_{charging_station_num}/charging_spot_{charging_spot_num}/contacts", self.listener_callback, 10)


    def listener_callback(self, msg):
        if self.ver_drone_50 > 50:
            self.ver_drone_50 = 0
            if self.occupied:
                self.get_logger().info(f'Charging Spot: {self.charging_spot_num} on Station: {self.landpad_station_num} AVAILABLE')
            self.occupied = False
        self.drone_num_occupying = self.validate_string(msg.data)
        if self.drone_num_occupying > 0:
            if not self.occupied:
                self.get_logger().info(f'Drone {self.drone_num_occupying} Attached to Charging Spot: {self.charging_spot_num} on Station: {self.charging_station_num}')
                self.occupied = True
        else:
            self.ver_drone_50+=1
    
    def validate_string(self, input_string):
        match = re.search(r'iris_(\d+)', input_string)
        if match:
            return int(match.group(1))
        else:
            return 0