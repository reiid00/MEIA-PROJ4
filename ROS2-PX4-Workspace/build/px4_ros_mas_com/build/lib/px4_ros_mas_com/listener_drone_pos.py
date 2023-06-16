import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import SensorCombined
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

class DronePosListener(Node):

    def __init__(self, drone_num = 1):
        super().__init__('minimal_subscriber')
        self.drone_num = drone_num
        self.topic_drone = f"/px4_{drone_num}"

        qos_profile = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE )

        self.current_position = [0.0, 0.0, 0.0]

        self.vehicle_local_pos_listener_ = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.listener_callback, qos_profile)


    def listener_callback(self, msg):
        self.current_position = [msg.x, msg.y, msg.z]
        self.get_logger().info(f"Current pos: {self.current_position}")



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = DronePosListener()
    rclpy.spin(minimal_subscriber)

if __name__ == '__main__':
    main()