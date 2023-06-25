
# Define constants and configurations
from enum import Enum

# Define internal configurations
HEIGHT_RANGE = (150, 200) # Height range allowed for drones, in meters
OPTIMUM_BATTERY_RANGE = (20, 80) # Optimum battery range for drones, in percentage
BATTERY_CHARGING_RATE = 5 # Battery gained per second when charging, in percentage
BATTERY_LOSS_RATE = 5 # Battery loss per second when active/moving, in percentage
NUM_DRONES = 4  # Number of drone agents
NUM_CHARGING_STATIONS = 4  # Number of charging stations
NUM_CHARGING_PADS_PER_CHARGING_STATION = 4  # Number of charging pads per charging station
BASE_COORDINATE = 20.0
AGENT_NAMES = {
    "CUSTOMER": "customer_agent",
    "APP": "app_agent",
    "TRAFFIC_CONTROL_STATION": "traffic_control_station_agent",
    "CHARGING_CONTROL_STATION": "charging_control_station_agent",
    "DRONE": "drone_agent",
    "DISPATCHER": "dispatcher_agent",
    "ROS2_NODE": "ros2_node_agent"
}
XMPP_SERVER_URL = "192.168.1.91"

# Define external configurations
DISPATCHER_API_URL = "https://dispatcher.simulation/orders"
APP_API_URL = "http://localhost:5001"

# Define utils configurations

class OrderStatus(Enum):
    TO_BE_ASSIGNED = 1
    ASSIGNED = 2
    ON_THE_WAY_TO_DISPATCHER = 3
    ARRIVED_AT_DISPATCHER = 4
    COLLECTED_GOODS_AT_DISPATCHER = 5
    ON_THE_WAY_TO_CUSTOMER = 6
    ARRIVED_AT_CUSTOMER = 7
    DELIVERED_GOODS_TO_CUSTOMER = 8
    COMPLETED = 9

class DroneStatus(Enum):
    UNAVAILABLE = 1
    AVAILABLE = 2
    OCCUPIED = 3
    OCCUPIED_PENDING_CHARGING = 4
    PENDING_CHARGING = 5
    ON_THE_WAY_TO_CHARGING_STATION = 6
    CHARGING = 7

class ChargingStationStatus(Enum):
    UNAVAILABLE = 1
    AVAILABLE = 2

class ChargingStationSpotStatus(Enum):
    UNAVAILABLE = 1
    AVAILABLE = 2
    OCCUPIED = 3

class DroneTargetType(Enum):
    DISPATCHER = 1
    CUSTOMER = 2
    CHARGING_STATION = 3
    LAND_PAD = 4

class EnvironmentType(Enum):
    Development = 1
    Production = 2

# Define current environment
ENVIRONMENT = EnvironmentType.Development.value