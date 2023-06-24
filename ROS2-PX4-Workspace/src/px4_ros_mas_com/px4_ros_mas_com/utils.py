import sys
from common.config import BASE_COORDINATE

def generate_station_coordinates(x, y):
    return {
        1: (x + 1, y + 1),
        2: (x + 1, y - 1),
        3: (x - 1, y + 1),
        4: (x - 1, y - 1)
    }

def create_lp_cs_locations():
    lp_stations = [(BASE_COORDINATE, BASE_COORDINATE), 
                   (BASE_COORDINATE, -BASE_COORDINATE), 
                   (-BASE_COORDINATE, BASE_COORDINATE), 
                   (-BASE_COORDINATE, -BASE_COORDINATE)]

    cs_stations = [(BASE_COORDINATE, 0.0), 
                   (0.0, BASE_COORDINATE), 
                   (-BASE_COORDINATE, 0.0), 
                   (0.0, -BASE_COORDINATE)]

    land_pads = {i: generate_station_coordinates(x, y) for i, (x, y) in enumerate(lp_stations, 1)}
    charging_stations = {i: generate_station_coordinates(x, y) for i, (x, y) in enumerate(cs_stations, 1)}
    
    return land_pads, charging_stations

def get_charging_station_spot_coordinates(charging_station_id, spot_id, charging_stations):
    try:
        if charging_station_id not in charging_stations: raise ValueError("Invalid charging station ID")
    
        station_coordinates = charging_stations[charging_station_id]
        if spot_id not in station_coordinates: raise ValueError("Invalid spot ID for the given charging station")
        
        (latitude, longitude) = station_coordinates[spot_id]
        return latitude, longitude
    except ValueError as e:
        print("Error:", str(e))
        return 0.0, 0.0

def allocate_shortest_station(drone_pos_x, drone_pos_y , array_station):
    shortest_distance = sys.float_info.max
    station_to_allocate = array_station[0]
    for station in array_station:
        if (station.is_available()):
            station_x, station_y = station.get_location()
            distance = calculate_distance(drone_pos_x, drone_pos_y, station_x, station_y)
            if (distance < shortest_distance):
                shortest_distance = distance
                station_to_allocate = station
    return station_to_allocate

def calculate_distance(x1, y1, x2, y2):
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5