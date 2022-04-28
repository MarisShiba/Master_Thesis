from .road import Road
from copy import deepcopy
from .vehicle_generator import VehicleGenerator
from .traffic_signal import TrafficSignal
import random
import pandas as pd
import numpy as np

class Simulation:
    def __init__(self, config={}):
        # Set default configuration
        self.set_default_config()

        # Update configuration
        for attr, val in config.items():
            setattr(self, attr, val)

    def set_default_config(self):
        self.t = 0.0            # Time keeping
        self.frame_count = 0    # Frame count keeping
        self.dt = 1/50          # Simulation time step
        self.roads = []         # Array to store roads
        self.generators = []
        self.traffic_signals = []
        self.vehicle_limit = 10
        self.records = pd.DataFrame(columns=['Vehicle_label', 'Road_order',
                                             'Total_time', 'Leading_vehicles',
                                             'Stopped_time', 'Gamma',
                                             'Eta'])
        self.all_routes = []
        self.vehicle_preferences = {}
        self.stop_condition = []
        self.gamma_mean = 1
        self.gamma_var = 0.5
        self.eta_mean = 1
        self.eta_var = 0.5

    def create_road(self, start, end, slow_factor=1, stop_distance=50, wait_time=1):
        road = Road(start, end, slow_factor, stop_distance, wait_time)
        self.roads.append(road)
        # self.wait_times.append(road.wait_time)
        return road

    def create_roads(self, road_list):
        for road in road_list:
            self.create_road(*road)

    def create_gen(self, config={}):
        gen = VehicleGenerator(self, config)
        self.generators.append(gen)
        return gen

    def create_signal(self, roads, config={}):
        roads = [[self.roads[i] for i in road_group] for road_group in roads]
        sig = TrafficSignal(roads, config)
        self.traffic_signals.append(sig)
        return sig

    def update(self):
        self.stop_condition = []
        # Update every road
        for road in self.roads:
            self.records = road.update(self.dt, self.records)

        # Add vehicles
        for gen in self.generators:
            gen.update()

        for signal in self.traffic_signals:
            signal.update(self)

        next_road_indices = []
        road_list = list(range(len(self.roads)))
        random.shuffle(road_list)
        for i in road_list:
            road = self.roads[i]
            # try:
            #     print(f"Current time is {self.t}, Current road's index is {vehicle.path[vehicle.current_road_index]}, Next road index is {vehicle.path[vehicle.current_road_index+1]}")
            # except:
            #     print("Blah")

            # If road has no vehicles, continue
            if len(road.vehicles) == 0:
                if self.generators[0].vehicle_cnt == 0:
                    self.stop_condition.append(0)
                continue
            else:
                self.stop_condition.append(0)

            vehicle = road.vehicles[0]

            # If this is not the last road of the vehicle's paths
            if vehicle.current_road_index + 1 < len(vehicle.path):
                next_road_index = vehicle.path[vehicle.current_road_index+1]
                wait_time = self.roads[next_road_index].wait_time
                # print(f"Vehicle label is {vehicle.label}, wait time is {wait_time}")

                if next_road_index not in next_road_indices:
                    self.roads[next_road_index].wait_time -= self.dt
                else:
                    next_road_indices.append(next_road_index)
                # print(f"Next road's start is {self.roads[next_road_index].start}, wait time is {self.roads[next_road_index]._wait_time}")

                # The vehicle will stop if it enters the stop zone while the next lane
                # is still occupied and there is not enough safe distance
                if vehicle.x >= road.length:

                    cond_1 = (len(self.roads[next_road_index].vehicles) == 0) or (self.roads[next_road_index].vehicles[-1].x >= (vehicle.s0 + vehicle.l))
                    cond_2 = wait_time <= 0

                    if cond_1 and cond_2:
                        self.roads[next_road_index].wait_time = self.roads[next_road_index]._wait_time

                        vehicle.current_road_index += 1
                        new_vehicle = deepcopy(vehicle)
                        new_vehicle.x = 0

                        if len(self.roads[next_road_index].vehicles)>0 and self.roads[next_road_index].vehicles[-1].v==0:
                            new_vehicle.v = 0
                        else:
                            new_vehicle.v = self.roads[next_road_index].slow_factor * new_vehicle._v_max
                        new_vehicle.v_max = self.roads[next_road_index].slow_factor * new_vehicle._v_max
                        new_vehicle.unstop()

                        self.roads[next_road_index].vehicles.append(new_vehicle)
                        self.roads[next_road_index].driven = True
                        # Remove it from its road
                        road.vehicles.popleft()

                    elif cond_2:
                        self.roads[next_road_index].wait_time = self.roads[next_road_index]._wait_time
                        vehicle.stop()
                        vehicle.v = 0
                    else:
                        vehicle.stop()
                        vehicle.v = 0

                else:
                    if self.roads[next_road_index].driven == True and wait_time <= 0:
                        self.roads[next_road_index].wait_time = self.roads[next_road_index]._wait_time
                    continue
            else:
                if vehicle.x >= road.length:
                    road.vehicles.popleft()
                else:
                    continue

        # Increment time
        self.t += self.dt

        self.frame_count += 1

    def run(self, steps):
        for _ in range(steps):
            self.update()
