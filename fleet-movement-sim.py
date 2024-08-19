import json
import geojson
import pandas as pd
import numpy as np
import time
from scipy.stats import truncnorm
from concurrent.futures import ThreadPoolExecutor
from typing import List
from shapely.geometry import shape, LineString
from geopy.distance import geodesic
from pyproj import Geod


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
                 start: Start, stops: List[Stop], max_speed, max_acceleration, max_deceleration):
        self.name = name
        self.device_mac = mac
        self.route_name = route
        self.route_direction = direction.lower()
        self.start = start
        self.stops = stops  # list of stops - pop first stop when reached
        self.v_max = max_speed
        self.a_max = max_acceleration
        self.dec_max = max_deceleration

        self.route = None
        self.df = None
        self.v_c = 0.0
        self.v_t = 0.0
        self.a = 0.0
        self.loc_section_index = 0  # index of the point-to-point section on which the train is currently located
        self.loc_coords = (0.0, 0.0)  # current location coordinates
        self.bearing = 0.0  # current bearing
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
        Max Deceleration: {self.dec_max}
        '''

    def setRouteDataFrame(self, route: LineString):
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
        self.en_route = True
        self.loc_section_index = self.start.df_index
        self.loc_coords = self.start.coordinates
        print(f'{self.name} is en route')
        while self.en_route:
            self.setTargetSpeed()
            self.setAcceleration()
            # print('Vc:', self.v_c, 'Vt:', self.v_t, 'A', self.a)
            time.sleep(5)
            # print('After 5 seconds')
            self.moveTrain(5)
            self.cltt += 5
            if self.isCurrentLocAStop():
                self.stopTrain()

    def calcDistanceToNextStop(self):
        # 10 meters buffer
        return 10 + self.calcCoordToIndexDistance(self.loc_coords, self.stops[0].df_index)

    def calcIndexToIndexDistance(self, index1, index2):
        return abs(self.df['CumulativeDistance'][index2] - self.df['CumulativeDistance'][index1])

    def calcCoordToIndexDistance(self, coord: tuple, index):
        """
        Gets distance between a coordinate and a future location index
        :param coord: coordinates
        :param index: upcoming location index
        :return: distance in meters
        """
        coord_point = (coord[1], coord[0])
        if index > self.loc_section_index:
            next_index_point = (self.df['Lat'][self.loc_section_index + 1], self.df['Long'][self.loc_section_index + 1])
            return geodesic(coord_point, next_index_point).meters + self.calcIndexToIndexDistance(self.loc_section_index + 1, index)
        elif index < self.loc_section_index:
            next_index_point = (self.df['Lat'][self.loc_section_index - 1], self.df['Long'][self.loc_section_index - 1])
            return geodesic(coord_point, next_index_point).meters + self.calcIndexToIndexDistance(self.loc_section_index - 1, index)
        else:
            # distance from current section start to given coordinates within the section
            index_point = (self.df['Lat'][index], self.df['Long'][index])
            return geodesic(coord_point, index_point).meters

    def calcCoordToCoordDistance(self, coord1: tuple, coord2: tuple):
        """
        Gets distance between two coordinates
        :param coord1: first coordinates
        :param coord2: second coordinates
        :return: distance in meters
        """
        coord1_point = (coord1[1], coord1[0])
        coord2_point = (coord2[1], coord2[0])
        return geodesic(coord1_point, coord2_point).meters

    def setTargetSpeed(self):
        s = self.calcDistanceToNextStop()
        if self.cltt >= self.stops[0].leg_duration_mins * 60 and s > 0:
            self.v_t = self.v_max
        elif s == 0:
            self.v_t = 0.0
        else:  # s = vt at constant speed
            t = self.stops[0].leg_duration_mins * 60 - self.cltt
            v = s / t
            self.v_t = round(v, 3) if v < self.v_max else self.v_max

    def setAcceleration(self):
        s = self.calcDistanceToNextStop()
        d_break = self.calcBreakingDistanceForSpeed(self.v_c)
        if s == 0:
            self.a = 0.0
        elif s < d_break:  # v^2 = u^2 + 2as
            u = self.v_c
            self.a = - round((u ** 2) / (2 * s), 5)
        elif self.v_c < self.v_t:
            # v = u + at
            v, u, t = self.v_t, self.v_c, 5
            a = (v - u) / t
            self.a = a if a < self.a_max else self.a_max
        else:
            self.a = 0.0

    def moveTrain(self, t):
        s = self.calcDistanceTravelledInTime(t)
        self.v_c = self.calcSpeedAfterTime(t)
        # print('Total time: ', self.cltt)
        # print('Distance traveled: ', s, 'Vc: ', self.v_c)
        # print('Distance to next stop: ', self.calcDistanceToNextStop())
        i, (long, lat) = self.findNewLocationCoordsAndSectionIndex(s)
        self.loc_section_index = i
        self.loc_coords = (long, lat)
        # print(i, long, lat)
        # print(self.bearing)
        # print()

    def calcDistanceTravelledInTime(self, t):  # s = ut + 0.5at^2
        u, a = self.v_c, self.a
        return (u * t) + (0.5 * a * (t ** 2))

    def calcSpeedAfterTime(self, t):  # v = u + at
        a = self.a
        v = self.v_c + (a * t)
        return v if v > 0 else 0.0

    def findNewLocationCoordsAndSectionIndex(self, s):
        """
        Gets index and coordinates of new location s meters away from current location
        :param s: distance from current location
        :return: index of new location section and coordinates of new location
        """
        if self.route_direction == 'up':
            for i in range(self.loc_section_index + 1, self.stops[0].df_index + 1):
                if self.calcCoordToIndexDistance(self.loc_coords, i) >= s:
                    new_section_index = i-1
                    if new_section_index != self.loc_section_index:
                        d = s - self.calcCoordToIndexDistance(self.loc_coords, new_section_index)
                    else:
                        d = self.calcCoordToIndexDistance(self.loc_coords, self.loc_section_index) + s
                    coords = self.getNewLocationCoords(new_section_index, d)
                    break
            else:
                new_section_index = self.stops[0].df_index
                coords = self.stops[0].coordinates
        else:
            for i in range(self.loc_section_index - 1, self.stops[0].df_index - 1, -1):
                if self.calcCoordToIndexDistance(self.loc_coords, i) >= s:
                    new_section_index = i+1
                    if new_section_index != self.loc_section_index:
                        d = s - self.calcCoordToIndexDistance(self.loc_coords, new_section_index)
                    else:
                        d = self.calcCoordToIndexDistance(self.loc_coords, self.loc_section_index) + s
                    coords = self.getNewLocationCoords(new_section_index, d)
                    break
            else:
                new_section_index = self.stops[0].df_index
                coords = self.stops[0].coordinates

        # coords = self.snapToRoute(coords, new_section_index)
        return new_section_index, coords

    def getNewLocationCoords(self, index, d):
        """
        Gets coordinates of location d meters away from start of section on the section
        :param index: index of new location section
        :param d: distance from start of next location section
        :return: coordinates of new location
        """
        if index == self.stops[-1].df_index:
            return self.df['Lat'][index], self.df['Long'][index]

        if self.route_direction == 'up':
            if index == len(self.df) - 1:
                return self.df['Long'][index], self.df['Lat'][index]
            point1 = (self.df['Lat'][index], self.df['Long'][index])
            point2 = (self.df['Lat'][index + 1], self.df['Long'][index + 1])
        else:
            if index == 0:
                return self.df['Long'][index], self.df['Lat'][index]
            point1 = (self.df['Lat'][index], self.df['Long'][index])
            point2 = (self.df['Lat'][index - 1], self.df['Long'][index - 1])

        geod = Geod(ellps='WGS84')
        az12, az21, distance = geod.inv(point1[1], point1[0], point2[1], point2[0])
        point = geodesic(kilometers=d / 1000).destination(point1, az12)
        self.bearing = az12
        return round(point.longitude, 6), round(point.latitude, 6)

    def isCurrentLocAStop(self):
        """
        Checks if current location section is a stop
        :return: True if current location is a stop, False otherwise
        """
        if self.route_direction == 'up':
            return self.loc_section_index >= self.stops[0].df_index
        else:
            return self.loc_section_index <= self.stops[0].df_index

    def stopTrain(self):
        stop_duration_mins = self.stops[0].stop_duration_mins
        print(f'Stopping at {self.stops[0].name} for {stop_duration_mins} minutes')
        self.cltt = 0
        self.v_c = 0.0
        self.v_t = 0.0
        self.a = 0.0
        time.sleep(stop_duration_mins * 60)
        self.stops.pop(0)
        if len(self.stops) == 0:
            self.en_route = False

    def calcBreakingDistanceForSpeed(self, u):
        # v^2 = u^2 + 2as where v = 0
        return (u ** 2) / (2 * self.dec_max)

    def printStatus(self):
        print(f'{self.name}: {self.loc_coords}  Speed: {round(self.v_c, 3)}  Bearing: {self.bearing}')

    def getStatus(self):
        """
        Get the current status of the train
        :return: name, longitude, latitude, speed, bearing
        """
        return self.name, self.en_route, self.loc_coords[0], self.loc_coords[1], round(self.v_c, 3), round(self.bearing, 3)


class GPS:
    def __init__(self, trains: List[Train]):
        self.trains = trains

    def start(self):
        pool = ThreadPoolExecutor(max_workers=len(self.trains) * 2)
        for train in self.trains:
            pool.submit(train.startJourney)
            pool.submit(self.processData, train)

    @staticmethod
    def addNoise(long: float, lat: float, bearing: float):
        """
        Adds noise to GPS coordinates.
        Gets a random eHPE(estimated horizontal position error) and
        generates a point within eHPE meters of the given coordinates.
        The actual point will be within the circle of radius eHPE meters around the returned coordinates.
        The generated point will have a higher probability of being
        in the direction of the bearing from the given coordinates.
        :param long: longitude
        :param lat: latitude
        :param bearing: direction the train is moving
        :return: longitude, latitude, eHPE
        """
        geod = Geod(ellps='WGS84')
        mu = 5  # mean
        sigma = 6  # standard deviation
        lower, upper = 1.0, 50.0  # bounds
        a, b = (lower - mu) / sigma, (upper - mu) / sigma  # Z
        ehpe = truncnorm.rvs(a, b, loc=mu, scale=sigma)
        angle = np.random.triangular(bearing - 180, bearing, bearing + 180)
        deviation = np.random.triangular(0, ehpe/2, ehpe)
        gps_point = geod.fwd(long, lat, angle, deviation)
        return gps_point[0], gps_point[1], ehpe

    def processData(self, train: Train):
        time.sleep(1)
        print(f'Processing data for {train.name}')
        name, en_route, long, lat, speed, bearing = train.getStatus()
        while en_route:
            long, lat, eHPE = self.addNoise(long, lat, bearing)
            print(name, round(long, 6), round(lat, 6), speed, bearing, round(eHPE, 3))
            self.send(name, round(long, 6), round(lat, 6), speed, bearing, round(eHPE, 3))
            time.sleep(10)

            name, en_route, long, lat, speed, bearing = train.getStatus()

    def send(self, name, long, lat, speed, bearing, eHPE):
        # TODO: send current location of trains every 30 seconds to server
        pass


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
                          train_data['maxAcceleration'], train_data['maxDeceleration'])

            for feature in railway_routes['features']:
                if feature['properties']['name'] == train_data['route']:
                    route = shape(feature['geometry'])
                    train.setRouteDataFrame(route)
                    break
            print(train)
            trains.append(train)

    gps = GPS(trains)
    gps.start()


if __name__ == "__main__":
    main()
