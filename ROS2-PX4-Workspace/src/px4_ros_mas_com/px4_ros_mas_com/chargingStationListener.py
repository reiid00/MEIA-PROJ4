import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String


class ChargingSpotListener(Node):

    def __init__(self, charging_station_num=1, charging_spot_num=1):
        super().__init__('charging_spot_listener')
        self.drone_num = drone_num

        self.drone_num_occupying=0

        self.occupied = False

        self.ver_drone_50=0

        self.drone_name = "iris"

        self.charging_spot_contact_listener_ = self.create_subscription(String, f"/charging_station_{charging_station_num}/charging_spot_{charging_spot_num}/contacts", self.listener_callback, 10)


    def listener_callback(self, msg):
        if ver_drone_50 > 50:
            ver_drone_50 = 0
            occupied = False
        contact_1, contact_2 = msg.data.split(",")
        drone_num = self.validate_drone(contact_1, contact_2)
        if drone_num > 0:
            occupied = True
            self.drone_num_occupying = drone_num
        else:
            ver_drone_50+=1
    
    def validate_drone(self, string, string2):
        if self.drone_name in string:
            return int(''.join(filter(str.isdigit, string)))
        elif self.drone_name in string2:
            return int(''.join(filter(str.isdigit, string2)))
        return 0
        