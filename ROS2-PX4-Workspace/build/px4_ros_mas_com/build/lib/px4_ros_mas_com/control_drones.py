import rclpy

from drone_offboard_control import DroneControl

def main(args=None):
    rclpy.init(args=args)

    num_drones = 5  # replace with your number of drones
    drone_controls = [DroneControl(i+1) for i in range(num_drones)]

    print(f"Starting offboard control node for {num_drones} drones...\n")

    try:
        rclpy.spin(drone_controls[0])  # You only need to spin on one node to keep the program from exiting.
    except KeyboardInterrupt:
        print("Stopping offboard control node...\n")
    finally:
        # Cleanup all nodes
        for drone_control in drone_controls:
            drone_control.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()