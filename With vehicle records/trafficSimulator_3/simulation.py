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

        next_road_indices = []
        road_list = list(range(len(self.roads)))
        random.shuffle(road_list)
        front_leads = [None] * len(road_list)
        # pre_speed = False

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

            j = 0
            while j < len(road.vehicles):
                vehicle = road.vehicles[j]

                # If this is not the last road of the vehicle's paths
                if vehicle.current_road_index + 1 < len(vehicle.path):
                    next_road_index = vehicle.path[vehicle.current_road_index+1]
                    wait_time = self.roads[next_road_index].wait_time

                    if (vehicle.x >= (road.length - road.stop_distance)) and (vehicle.x < road.length):
                        # Adjust the speed in advance
                        if vehicle.v_max != (self.roads[next_road_index].slow_factor*vehicle._v_max):
                            # print(f"Vehicle {vehicle.label} x: {round(vehicle.x, 2)}, vehicle path: {vehicle.current_road_index}, changing maximum speed from {vehicle.v_max}")

                            # pre_speed = True
                            vehicle.v_max = self.roads[next_road_index].slow_factor*vehicle._v_max
                            # print(f"To {vehicle.v_max}")
                        if vehicle.v <= 0:
                            vehicle.unstop()

                        if j*self.roads[next_road_index]._wait_time+wait_time > 0:
                            if (road.length - vehicle.x)/(vehicle.v_max*(j*self.roads[next_road_index]._wait_time+wait_time))>=1:
                                # print(f"Unstop vehicle {vehicle.label}!!")
                                vehicle.unstop()
                            else:
                                # print(f"Vehicle {vehicle.label} should be stopping at position {round(vehicle.x, 2)}")
                                # vehicle.stop(previous_sf=road.slow_factor)
                                vehicle.stop()
                                # print(f"Vehicle {vehicle.label} stopped status is {vehicle.stopped}")
                                # print(f"Vehicle {vehicle.label} has acceleration {round(vehicle.a, 2)}")

                    # For the leading vehicle of that road some special conditions are set
                    if j == 0:
                        if next_road_index not in next_road_indices:
                            self.roads[next_road_index].wait_time -= self.dt
                        else:
                            next_road_indices.append(next_road_index)

                        if wait_time<=0 and self.roads[next_road_index].driven == True:
                            self.roads[next_road_index].wait_time = self.roads[next_road_index]._wait_time

                        if len(self.roads[next_road_index].vehicles) != 0:
                            ghost_car = deepcopy(self.roads[next_road_index].vehicles[-1])
                            ghost_car.x += road.length
                            front_leads[i] = ghost_car

                        # When vehicle is out of road bounds
                        if vehicle.x >= road.length:

                            if wait_time <= 0:
                                # Update current road to next road
                                vehicle.current_road_index += 1

                                # Create a copy and reset some vehicle properties
                                new_vehicle = deepcopy(vehicle)
                                new_vehicle.x = 0
                                new_vehicle.unstop()

                                self.roads[next_road_index].vehicles.append(new_vehicle)
                                self.roads[next_road_index].driven = True

                                # Remove it from its road
                                road.vehicles.popleft()
                                j -= 1
                                # print(self.t)

                                ghost_car = deepcopy(new_vehicle)
                                ghost_car.x += road.length
                                front_leads[i] = ghost_car

                                self.roads[next_road_index].wait_time = self.roads[next_road_index]._wait_time
                            else:
                                vehicle.v_max = self.roads[next_road_index].slow_factor*vehicle._v_max
                                # print(f"Vehicle {vehicle.label} should stop to wait for {round(wait_time, 2)}s with speed {round(vehicle.v, 2)} and acceleration {round(vehicle.a)}")
                                # vehicle.stop(previous_sf=road.slow_factor)
                                vehicle.stop()
                                # print(f"Vehicle {vehicle.label} stopped status is {vehicle.stopped}")
                else:
                    if vehicle.x >= road.length:
                        road.vehicles.popleft()
                j += 1
            self.records = road.update(self.dt, self.records, front_leads[i])
            # if front_leads[i]!= None:
            #     print(f"Road number {i}, has vehicles {[v.label for v in road.vehicles]}, front_lead is {front_leads[i].label}")
            # else:
            #     print(f"Road number {i}, has vehicles {[v.label for v in road.vehicles]}, front_lead is {front_leads[i]}")

        # for i in range(len(self.roads)):
        #     road = self.roads[i]
        #     self.records = road.update(self.dt, self.records, front_leads[i])

        # Add vehicles
        for gen in self.generators:
            gen.update()

        for signal in self.traffic_signals:
            signal.update(self)

        # Increment time
        self.t += self.dt

        self.frame_count += 1

    def run(self, steps):
        for _ in range(steps):
            self.update()
