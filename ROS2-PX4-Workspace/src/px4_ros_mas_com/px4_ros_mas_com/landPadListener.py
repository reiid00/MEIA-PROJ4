import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String


class LandPadListener(Node):

    def __init__(self, landpad_station_num=1, landpad_num=1):
        super().__init__('land_pad_listener')

        self.drone_num_occupying=0

        self.occupied = False

        self.ver_drone_50=0

        self.drone_name = "iris"

        self.land_pad_contact_listener_ = self.create_subscription(String, f"/landpad_station_{landpad_station_num}/land_pad_{landpad_num}/contacts", self.listener_callback, 10)


    def listener_callback(self, msg):
        if self.ver_drone_50 > 50:
            self.ver_drone_50 = 0
            occupied = False
        contact_1, contact_2 = msg.data.split(",")
        
        self.drone_num_occupying = self.validate_drone(contact_1, contact_2)
        if self.drone_num_occupying > 0:
            print(f'Drone attached, Drone num: {self.drone_num_occupying}')
            occupied = True
        else:
            self.ver_drone_50+=1
    
    def validate_drone(self, string, string2):
        if self.drone_name in string:
            return int(''.join(filter(str.isdigit, string)))
        elif self.drone_name in string2:
            return int(''.join(filter(str.isdigit, string2)))
        return 0
        