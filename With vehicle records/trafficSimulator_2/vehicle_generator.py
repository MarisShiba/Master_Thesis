from .vehicle import Vehicle
from numpy.random import randint
import numpy as np

class VehicleGenerator:
    def __init__(self, sim, config={}):
        self.sim = sim
        self.vehicle_cnt = 0

        # Set default configurations
        self.set_default_config()

        # Update configurations
        for attr, val in config.items():
            setattr(self, attr, val)

        # Calculate properties
        self.init_properties()

    def set_default_config(self):
        """Set default configuration"""
        self.vehicle_rate = 20
        self.vehicles = {}
        self.vehicle_limit = 10
        self.last_added_time = 0
        self.vehicle_order = list(range(self.vehicle_limit))
        np.random.shuffle(self.vehicle_order)


    def init_properties(self):
        self.upcoming_vehicle = self.generate_vehicle()


    # def generate_vehicle(self):
    #     """Returns a random vehicle from self.vehicles with random proportions"""
    #     total = sum(pair[0] for pair in self.vehicles)
    #     r = randint(1, total+1)
    #     for (weight, config) in self.vehicles:
    #         r -= weight
    #         if r <= 0:
    #             config['label'] = self.vehicle_cnt
    #             return Vehicle(config)

    def generate_vehicle(self):
        try:
            vehicle = self.vehicle_order[self.vehicle_cnt]
            gamma = np.random.normal(self.sim.gamma_mean, self.sim.gamma_var, 1)[0]
            eta = np.random.normal(self.sim.eta_mean, self.sim.eta_var, 1)[0]

            # print(self.sim.vehicle_preferences)
            probs = self.sim.vehicle_preferences[self.vehicle_cnt]
            path_num = np.random.choice(len(self.sim.all_routes), size=1, p=probs)[0]

            config = self.vehicles
            config['path'] = self.sim.all_routes[path_num]
            config['label'] = self.vehicle_cnt
            config['gamma'] = gamma
            config['eta'] = eta

            return Vehicle(config)
        except:
            return None

    def update(self):
        """Add vehicles"""
        if self.vehicle_cnt < self.vehicle_limit and self.upcoming_vehicle!=None:
            if (self.sim.t - self.last_added_time >= 60 / self.vehicle_rate):
            # If time elasped after last added vehicle is
            # greater than vehicle_period; generate a vehicle
                road = self.sim.roads[self.upcoming_vehicle.path[0]]

                if len(road.vehicles) == 0\
                   or road.vehicles[-1].x > self.upcoming_vehicle.s0 + self.upcoming_vehicle.l:
                    # If there is space for the generated vehicle; add it
                    self.upcoming_vehicle.time_added = self.sim.t
                    road.vehicles.append(self.upcoming_vehicle)
                    # Reset last_added_time and upcoming_vehicle
                    self.last_added_time = self.sim.t

                    self.vehicle_cnt += 1
                    self.sim.records.loc[len(self.sim.records)] = [self.vehicle_cnt-1, [], 0, [], [], -99, -99]

                self.upcoming_vehicle = self.generate_vehicle()
