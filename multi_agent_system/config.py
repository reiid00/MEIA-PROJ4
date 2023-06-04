
# Define constants and configurations
from enum import Enum

# Define internal configurations
HEIGHT_RANGE = (150, 200) # Height range allowed for drones
NUM_DRONES = 3  # Number of drone agents
AGENT_NAMES = {
    "CUSTOMER": "customer_agent",
    "APP": "app_agent",
    "TRAFFIC_CONTROL_STATION": "traffic_control_station_agent",
    "DRONE": "drone_agent",
    "DISPATCHER": "dispatcher_agent"
}

# Define external configurations
DISPATCHER_API_URL = "https://dispatcher.simulation/orders"
APP_API_URL = "https://app.simulation/orders"

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
    OCCUPIED = 2
    CHARGING = 3