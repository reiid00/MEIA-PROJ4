
from geopy.distance import great_circle

def calculate_distance(self, location1, location2):
    # Check if any location is invalid
    if not location1 or not location2 or \
            "latitude" not in location1 or "longitude" not in location1 or \
            "latitude" not in location2 or "longitude" not in location2:
        return float("inf")

    point1 = (location1["latitude"], location1["longitude"])
    point2 = (location2["latitude"], location2["longitude"])

    distance = great_circle(point1, point2).meters
    return distance