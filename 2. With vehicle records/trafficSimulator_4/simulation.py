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
        self.round_number = 1
        self.records = pd.DataFrame(columns=['Vehicle_label', 'Road_order',
                                             'Total_time', 'Leading_vehicles',
                                             'Stopped_time', 'Stop_while_front'])
        # self.records = pd.DataFrame(columns=['Vehicle_label', 'Road_order',
        #                                      'Total_time', 'Leading_vehicles',
        #                                      'Stopped_time', 'Stop_while_front', 'Shortest_dist'])
        self.all_routes = []
        self.vehicle_preferences = {}
        self.stop_condition = []
        # self.gamma_mean = 1
        # self.gamma_var = 0.5
        # self.eta_mean = 1
        # self.eta_var = 0.5

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
        # compare_stoptime_df = pd.DataFrame(columns=['Vehicle_label', 'Entering_road', 'Stopped_time'])

        next_road_indices = []
        road_list = list(range(len(self.roads)))

        wait_dict = {}
        for p in road_list:
            road = self.roads[p]
            road.appended = False
            if len(road.vehicles) != 0:
                # print("Road number is", p)
                head_v = road.vehicles[0]
                # print(f"Head vehicle number is {head_v.label} and paths are {head_v.path}")
                if head_v.current_road_index + 1 < len(head_v.path):
                    idx = self.records.loc[self.records.Vehicle_label==head_v.label].index.values[0]
                    if self.records.at[idx,"Road_order"] != []:
                        stop_time_idx = self.records.at[idx,"Road_order"].index(p)
                        stop_time = self.records.at[idx,"Stop_while_front"][stop_time_idx]
                        if head_v.path[head_v.current_road_index+1] not in wait_dict.keys():
                            wait_dict[head_v.path[head_v.current_road_index+1]] = [head_v.label, stop_time]
                        if stop_time > wait_dict[head_v.path[head_v.current_road_index+1]][1]:
                            wait_dict[head_v.path[head_v.current_road_index+1]] = [head_v.label, stop_time]
            else:
                road.driven = False
                continue

        # if len(wait_dict)!=0:
        #     print(wait_dict)

        # for p in road_list:
        #     road = self.roads[p]
        #     road.appended = False
        #     try:
        #         head_v = road.vehicles[0]
        #         stop_time_idx = self.records.at[head_v.label,"Road_order"].index(p)
        #         stop_time = self.records.at[head_v.label,"Stopped_time"][stop_time_idx]
        #         compare_stoptime_df.loc[len(compare_stoptime_df)] = [head_v.label, head_v.path[head_v.path.index(p)+1], round(stop_time, 2)]
        #     except:
        #         continue

        # # To give priority to leading vehicles of each road that have waited the longest
        # wait_dict = {}
        # for q in road_list:
        #     try:
        #         max_stop = compare_stoptime_df.loc[compare_stoptime_df.Entering_road==q]['Stopped_time'].max()
        #         waiting_v = compare_stoptime_df.loc[compare_stoptime_df.Stopped_time==max_stop]['Vehicle_label'].values[0]
        #         wait_dict[q] = waiting_v
        #     except:
        #         continue

        random.shuffle(road_list)
        front_leads = [None] * len(road_list)
        # pre_speed = False

        for i in road_list:
            road = self.roads[i]

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
                        if j == 0:
                            vehicle.be_front = True
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
                        else:
                            vehicle.unstop()

                    # For the leading vehicle of that road some special conditions are set
                    if j == 0:
                        vehicle.be_front = True
                        
                        if next_road_index not in next_road_indices:
                            self.roads[next_road_index].wait_time -= self.dt
                        else:
                            next_road_indices.append(next_road_index)

                        wait_time = self.roads[next_road_index].wait_time
                        if wait_time<=0 and self.roads[next_road_index].driven == True:
                            self.roads[next_road_index].wait_time = self.roads[next_road_index]._wait_time

                        if len(self.roads[next_road_index].vehicles) != 0:
                            ghost_car = deepcopy(self.roads[next_road_index].vehicles[-1])
                            ghost_car.x += road.length
                            front_leads[i] = ghost_car

                        # When vehicle is out of road bounds
                        if vehicle.x >= road.length:
                            # cond = (wait_dict[vehicle.path[vehicle.current_road_index+1]] == vehicle.label) or (self.roads[next_road_index].driven==False)
                            # cond = True
                            # cond = (wait_dict[next_road_index][0] == vehicle.label) or (self.roads[next_road_index].driven==False)
                            if len(self.roads[next_road_index].vehicles)==0:
                                cond = True
                            elif self.roads[next_road_index].vehicles[-1].x>(vehicle.s0 + vehicle.l):
                                cond = True
                            else:
                                cond = False

                            if len(wait_dict)>0:
                                if wait_dict[next_road_index][0] == vehicle.label:
                                    cond = True
                            elif len(wait_dict)==0:
                                cond = True

                            if (wait_time <= 0) and (self.roads[next_road_index].appended != True) and cond:
                                # Update current road to next road
                                vehicle.current_road_index += 1

                                # Create a copy and reset some vehicle properties
                                new_vehicle = deepcopy(vehicle)
                                new_vehicle.x = 0
                                new_vehicle.be_front = False
                                new_vehicle.unstop()

                                self.roads[next_road_index].vehicles.append(new_vehicle)
                                self.roads[next_road_index].driven = True
                                self.roads[next_road_index].appended = True

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
