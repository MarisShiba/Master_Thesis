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
        self.vehicle_limit = 10 # Total number of vehicles
        self.round_number = 1
        self.records = pd.DataFrame(columns=['Vehicle_label', 'Road_order',
                                             'Total_time', 'Leading_vehicles',
                                             'Stopped_time', 'Stop_while_front'])
        self.all_routes = []
        self.vehicle_preferences = {}
        self.stop_condition = []
        self.wait_dict = {}

    def create_road(self, start, end, slow_factor=1, stop_distance=50, wait_time=1):
        road = Road(start, end, slow_factor, stop_distance, wait_time)
        self.roads.append(road)
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

        wait_dict = self.wait_dict
        for p in road_list:
            road = self.roads[p]
            # Before starting every new frame, each road should have a False
            # status of having been appended
            road.appended = False

            # The wait_dict variable is used for implementing a mechanism where,
            # when multiple leading vehicles from different roads want to enter
            # the same road as their next road, they are assigned different
            # priorities based on the amount of time they spent waiting in the
            # stop zone of the road they're on right now.
            if len(road.vehicles) != 0:
                head_v = road.vehicles[0]

                # Checking if the head vehicle has a next road to enter
                if head_v.current_road_index + 1 < len(head_v.path):
                    # Identify the dataframe index number that the vehicle's
                    # record is in
                    idx = self.records.loc[self.records.Vehicle_label==head_v.label].index.values[0]

                    if self.records.at[idx,"Road_order"] != []:
                        stop_time_idx = self.records.at[idx,"Road_order"].index(p)
                        # Extract the stop time that they vehicle has accumulated
                        # while being the head vehicle of the road after they
                        # enter the stop zone
                        stop_time = self.records.at[idx,"Stop_while_front"][stop_time_idx]

                        if head_v.path[head_v.current_road_index+1] not in wait_dict.keys():
                            wait_dict[head_v.path[head_v.current_road_index+1]] = {}

                        # The keys of the wait_dict is the road label of the next
                        # road that the head vehicle will enter
                        # The values of the wait_dict are dictionaries recording
                        # the head vehicle's label as keys and their stopped time
                        # so far as values
                        wait_dict[head_v.path[head_v.current_road_index+1]][head_v.label] = stop_time

                        # After updating the wait_dict with the head vehicle's record,
                        # each of its element pair is sorted according to the
                        # stop time amount, which is stored as the value of the
                        # of each wait_dict's value
                        wait_dict[head_v.path[head_v.current_road_index+1]] = dict(sorted(wait_dict[head_v.path[head_v.\
                                                                              current_road_index+1]].items(),\
                                                                              key=lambda item: item[1]))
            else:
                road.driven = False
                continue

        random.shuffle(road_list)
        # The front_leads variable is set to serve as adding a "ghost vehicle"
        # that is the same as the vehicle to the road that the vehicle just left
        # In this way, the previous road's new head vehicle can still adjust
        # their driving behavior based on this ghost vehicle without it still
        # being considered part of the current road.
        front_leads = [None] * len(road_list)

        for i in road_list:
            road = self.roads[i]

            # The condition for stopping the simulation is triggered when at
            # least 1 vehicle has been generated and all the roads have no
            # vehicles in them.
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

                    if j == 0:
                        if next_road_index not in next_road_indices:
                            # For every frame, all roads' wait time will be reduced
                            self.roads[next_road_index].wait_time -= self.dt

                            next_road_indices.append(next_road_index)

                    wait_time = self.roads[next_road_index].wait_time

                    # When vehicles have entered the stop zone:
                    if (vehicle.x >= (road.length - road.stop_distance)) and (vehicle.x < road.length):
                        if j == 0:
                            # Mark the lead vehicle
                            vehicle.be_front = True

                            # Estimate if the vehicle can reach the end of the
                            # road before the wait time is over--if it can, then
                            # it should start to stop; otherwise no.
                            if (road.length - vehicle.x)/(vehicle.v_max*wait_time)>=1:
                                if len(self.roads[next_road_index].vehicles)>0:
                                    if self.roads[next_road_index].vehicles[-1].x>(vehicle.s0 + vehicle.l):
                                        vehicle.unstop()
                                else:
                                    vehicle.unstop()
                            else:
                                vehicle.stop()

                        # Adjust the speed in advance if the next road has a different
                        # maximum speed
                        if vehicle.v_max != (self.roads[next_road_index].slow_factor*vehicle._v_max):
                            vehicle.v_max = self.roads[next_road_index].slow_factor*vehicle._v_max

                        # Estimate if the vehicle can reach the end of the road
                        # before the wait time is over--if it can, then it
                        # should start to stop; otherwise no.
                        if j*self.roads[next_road_index]._wait_time+wait_time > 0 and j != 0:
                            if (road.length - vehicle.x)/((vehicle.v+vehicle.v_max)/2*(j*self.roads[next_road_index]._wait_time+wait_time))>=1:
                                vehicle.unstop()
                            else:
                                vehicle.stop()
                        else:
                            vehicle.unstop()

                    # For the leading vehicle of that road some special conditions are set
                    if j == 0:
                        wait_time = self.roads[next_road_index].wait_time

                        if wait_time<=0 and self.roads[next_road_index].driven == True:
                            # self.roads[next_road_index].wait_time = self.roads[next_road_index]._wait_time
                            vehicle.unstop()

                        # Add a ghost vehicle which is the same as the last vehicle
                        # of the next road to help adjust the driving of the head car
                        if len(self.roads[next_road_index].vehicles) != 0:
                            ghost_car = deepcopy(self.roads[next_road_index].vehicles[-1])
                            ghost_car.x += road.length
                            front_leads[i] = ghost_car

                        # When vehicle is out of road bounds
                        if vehicle.x >= road.length:
                            if len(self.roads[next_road_index].vehicles)==0:
                                cond = True
                            elif self.roads[next_road_index].vehicles[-1].x>(1.5*vehicle.s0 + vehicle.l):
                                cond = True
                            else:
                                cond = False

                            if len(wait_dict[next_road_index])>0:
                                if list(wait_dict[next_road_index].keys())[-1] == vehicle.label:
                                    cond = True
                            else:
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
                                del wait_dict[next_road_index][vehicle.label]
                                self.wait_dict = wait_dict
                                j -= 1

                                ghost_car = deepcopy(new_vehicle)
                                ghost_car.x += road.length
                                front_leads[i] = ghost_car

                                self.roads[next_road_index].wait_time = self.roads[next_road_index]._wait_time
                            else:
                                vehicle.v_max = self.roads[next_road_index].slow_factor*vehicle._v_max
                                vehicle.stop()
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

        # Increment time and frame
        self.t += self.dt
        self.frame_count += 1

    def run(self, steps):
        for _ in range(steps):
            self.update()
