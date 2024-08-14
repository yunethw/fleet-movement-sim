# 10 Trains
# Traveling along predefined routes
from typing import List
import json
import geojson
from shapely.geometry import shape, Point, LineString
from geopy.distance import geodesic


class Start:
    def __init__(self, name, time, coordinates: tuple):
        self.name = name
        self.time = time
        self.coordinates = coordinates

    def __str__(self):
        return f'{self.name} {self.time} {self.coordinates}'


class Stop:
    def __init__(self, name, leg_duration_mins, stop_duration_mins, coordinates: tuple):
        self.name = name
        self.leg_duration_mins = leg_duration_mins
        self.stop_duration_mins = stop_duration_mins
        self.coordinates = coordinates

    def __str__(self):
        return f'{self.name} {self.coordinates}'


class Train:
    route: LineString
    Vc = 0.0  # Current velocity
    Vt = 0.0  # Target velocity
    Vmax = 0.0  # Maximum velocity
    A = 0.0  # Acceleration
    Dbreak = 0.0  # Breaking distance

    def __init__(self, name: str, mac: str, route: str,
                 start: Start, stops: List[Stop], max_speed, max_acceleration, breaking_distance):
        self.name = name
        self.device_mac = mac
        self.route_name = route
        self.start = start
        self.stops = stops
        self.Vmax = max_speed
        self.A = max_acceleration
        self.Dbreak = breaking_distance

    def setRouteLineString(self, route: LineString):
        self.route = route

    def __str__(self):
        return f'''Train: {self.name}
        Device MAC: {self.device_mac}
        Route: {self.route_name}
        Start: {str(self.start)}
        Stops: {', '.join([str(stop) for stop in self.stops])}
        Max Speed: {self.Vmax}
        Max Acceleration: {self.A}
        Breaking Distance: {self.Dbreak}
        '''


def main():
    railway_routes = geojson.load(open('railway-routes.geojson'))
    for feature in railway_routes['features']:
        total_distance = 0.0
        coords = feature['geometry']['coordinates']
        for i in range(len(coords) - 1):
            point1 = (coords[i][1], coords[i][0])
            point2 = (coords[i + 1][1], coords[i + 1][0])
            total_distance += geodesic(point1, point2).kilometers
        print(feature['properties']['name'], feature['geometry']['type'], len(feature['geometry']['coordinates']), 'coordinates', round(total_distance, 3), 'km')

    trains = []
    with open('fleet.json') as f:
        data = json.load(f)
        for train_data in data['collection']:
            start = Start(train_data['start']['name'], train_data['start']['time'], train_data['start']['coordinates'])
            stops = []

            for stop_data in train_data['stops']:
                stop = Stop(stop_data['name'], stop_data['legDurationMins'], stop_data['stopDurationMins'], stop_data['coordinates'])
                stops.append(stop)

            train = Train(train_data['name'], train_data['deviceMAC'], train_data['route'], start, stops, train_data['maxSpeed'], train_data['maxAcceleration'], train_data['breakingDistance'])

            for feature in railway_routes['features']:
                if feature['properties']['name'] == train_data['route']:
                    route = shape(feature['geometry'])
                    train.setRouteLineString(route)
                    break
            print(train)
            trains.append(train)


if __name__ == "__main__":
    main()
