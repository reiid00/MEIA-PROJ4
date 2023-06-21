
BASE_COORDINATE = 20.0
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