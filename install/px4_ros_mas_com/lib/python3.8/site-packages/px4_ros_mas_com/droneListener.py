import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition


class DroneListener(Node):

    def __init__(self, drone_num = 1):
        super().__init__('minimal_subscriber')
        self.drone_num = drone_num
        topic_drone = f"/px4_{drone_num}"

        qos_profile = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE )

        self.current_position = [0.0, 0.0, 0.0]

        self.vehicle_local_pos_listener_ = self.create_subscription(VehicleLocalPosition, f"{topic_drone}/fmu/out/vehicle_local_position", self.listener_callback, qos_profile)


    def listener_callback(self, msg):
        self.current_position = [msg.x, msg.y, msg.z]