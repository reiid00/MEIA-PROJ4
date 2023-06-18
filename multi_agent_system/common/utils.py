
from config import ENVIRONMENT, EnvironmentType
from geopy.distance import great_circle


def calculate_distance(location1, location2):
    # Check if any location is invalid
    if not location1 or not location2 or \
            "latitude" not in location1 or "longitude" not in location1 or \
            "latitude" not in location2 or "longitude" not in location2:
        return float("inf")

    if ENVIRONMENT == EnvironmentType.Development.value:
        # Development environment: Use Euclidean distance with latitude as x and longitude as y
        x1, y1 = location1["latitude"], location1["longitude"]
        x2, y2 = location2["latitude"], location2["longitude"]
        distance = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    else:
        # Production environment: Use geopy's great_circle to calculate the distance
        point1 = (location1["latitude"], location1["longitude"])
        point2 = (location2["latitude"], location2["longitude"])
        distance = great_circle(point1, point2).meters

    return distance