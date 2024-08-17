# 10 Trains
# Traveling along predefined routes
import json
import geojson
import pandas as pd
import numpy as np
from typing import List
from shapely.geometry import shape, Point, LineString
from geopy.distance import geodesic


class Start:

    def __init__(self, name, start_time, coordinates: tuple):
        self.name = name
        self.time = start_time
        self.coordinates = coordinates
        self.df_index = 0

    def __str__(self):
        return f'{self.df_index} {self.name} {self.time} {self.coordinates}'

    def setIndex(self, index):
        self.df_index = index


class Stop:

    def __init__(self, name, leg_duration_mins, stop_duration_mins, coordinates: tuple):
        self.name = name
        self.leg_duration_mins = leg_duration_mins
        self.stop_duration_mins = stop_duration_mins
        self.coordinates = coordinates
        self.df_index = 0

    def __str__(self):
        return f'{self.df_index} {self.name} {self.coordinates}'

    def setIndex(self, index):
        self.df_index = index


class Train:

    def __init__(self, name: str, mac: str, route: str, direction: str,
                 start: Start, stops: List[Stop], max_speed, max_acceleration, breaking_distance):
        self.name = name
        self.device_mac = mac
        self.route_name = route
        self.route_direction = direction.lower()
        self.start = start
        self.stops = stops  # list of stops - pop first stop when reached
        self.v_max = max_speed
        self.a_max = max_acceleration
        self.d_break = breaking_distance

        self.route = None
        self.df = None
        self.v_c = 0.0
        self.v_t = 0.0
        self.a = 0.0
        self.loc_section_index = 0  # index of the point-to-point section on which the train is currently located
        self.loc_coords = (0.0, 0.0)  # current location coordinates
        self.cltt = 0.0  # current leg travel time
        self.en_route = False

    def __str__(self):
        return f'''Train: {self.name}
        Device MAC: {self.device_mac}
        Route: {self.route_name} {self.route_direction}
        Start: {str(self.start)}
        Stops: {', '.join([str(stop) for stop in self.stops])}
        Max Speed: {self.v_max}
        Max Acceleration: {self.a_max}
        Breaking Distance: {self.d_break}
        '''

    def setRouteLineString(self, route: LineString):
        self.route = route
        self.df = pd.DataFrame(route.coords, columns=['Long', 'Lat'])
        gap = []
        for i in range(len(route.coords)):
            if i == 0:
                gap.append(0.0)
            else:
                point1 = (route.coords[i - 1][1], route.coords[i - 1][0])
                point2 = (route.coords[i][1], route.coords[i][0])
                gap.append(round(geodesic(point1, point2).meters, 3))

            if self.start.coordinates == route.coords[i]:
                self.start.setIndex(i)
            for stop in self.stops:
                if stop.coordinates == route.coords[i]:
                    stop.setIndex(i)
                    break

        cum_dist = [round(x, 3) for x in np.cumsum(gap)]
        self.df['Gap'] = gap
        self.df['CumulativeDistance'] = cum_dist
        # print(self.df.head(10))
        # print(self.df.tail(10))

    def startJourney(self):
        self.loc_index = self.start.index
        print(self.distanceToNextStop())

    def distanceToNextStop(self):
        if self.loc_index is None:
            return None
        if str.lower(self.route_direction) == 'up':
            for stop in self.stops:
                if stop.index > self.loc_index:
                    return self.df['CumulativeDistance'][stop.index] - self.df['CumulativeDistance'][self.loc_index]
            return None
        elif str.lower(self.route_direction) == 'down':
            for stop in self.stops:
                if stop.index < self.loc_index:
                    return self.df['CumulativeDistance'][self.loc_index] - self.df['CumulativeDistance'][stop.index]
            return None
        return None


def main():
    railway_routes = geojson.load(open('railway-routes.geojson'))
    for feature in railway_routes['features']:
        total_distance = 0.0
        coords = feature['geometry']['coordinates']
        # for i in range(len(coords) - 1):
        #     point1 = (coords[i][1], coords[i][0])
        #     point2 = (coords[i + 1][1], coords[i + 1][0])
        #     total_distance += geodesic(point1, point2).kilometers
        print(feature['properties']['name'], feature['geometry']['type'],
              len(feature['geometry']['coordinates']), 'coordinates', round(total_distance, 3), 'km')

    trains = []
    with open('fleet.json') as f:
        data = json.load(f)
        for train_data in data['collection']:
            start_coords = tuple(round(c, 6) for c in train_data['start']['coordinates'])
            start = Start(train_data['start']['name'], train_data['start']['time'], start_coords)

            stops = []
            for stop_data in train_data['stops']:
                stop_coords = tuple(round(c, 6) for c in stop_data['coordinates'])
                stop = Stop(stop_data['name'], stop_data['legDurationMins'],
                            stop_data['stopDurationMins'], stop_coords)
                stops.append(stop)

            train = Train(train_data['name'], train_data['deviceMAC'], train_data['route'],
                          train_data['routeDirection'], start, stops, train_data['maxSpeed'],
                          train_data['maxAcceleration'], train_data['breakingDistance'])

            for feature in railway_routes['features']:
                if feature['properties']['name'] == train_data['route']:
                    route = shape(feature['geometry'])
                    train.setRouteLineString(route)
                    break
            print(train)
            trains.append(train)

    for train in trains:
        train.startJourney()


if __name__ == "__main__":
    main()
