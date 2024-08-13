# 10 Trains
# Traveling along predefined routes
from typing import List
import json


class Start:
    def __init__(self, name, time, coordinates: tuple):
        self.name = name
        self.time = time
        self.coordinates = coordinates


class Stop:
    def __init__(self, name, leg_duration_mins, stop_duration_mins, coordinates: tuple):
        self.name = name
        self.leg_duration_mins = leg_duration_mins
        self.stop_duration_mins = stop_duration_mins
        self.coordinates = coordinates

    def __str__(self):
        return f'{self.name} {self.coordinates}'


class Train:
    def __init__(self, name: str, route: str, start: Start, stops: List[Stop], max_speed, max_acceleration, breaking_distance):
        self.name = name
        self.route = route
        self.start = start
        self.stops = stops
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        self.breaking_distance = breaking_distance

    def __str__(self):
        return f'''Train: {self.name}
        Route: {self.route}
        Start: {self.start.name} {self.start.time} {self.start.coordinates}
        Stops: {', '.join([str(stop) for stop in self.stops])}
        Max Speed: {self.max_speed}
        Max Acceleration: {self.max_acceleration}
        Breaking Distance: {self.breaking_distance}
        '''


def main():
    trains = []
    with open('fleet.json') as f:
        data = json.load(f)
        for train_data in data['collection']:
            start = Start(train_data['start']['name'], train_data['start']['time'], train_data['start']['coordinates'])
            stops = []
            for stop_data in train_data['stops']:
                stop = Stop(stop_data['name'], stop_data['legDurationMins'], stop_data['stopDurationMins'], stop_data['coordinates'])
                stops.append(stop)
            train = Train(train_data['name'], train_data['route'], start, stops, train_data['maxSpeed'], train_data['maxAcceleration'], train_data['breakingDistance'])
            print(train)
            trains.append(train)


if __name__ == "__main__":
    main()
