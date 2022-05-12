import numpy as np

class Vehicle:
    def __init__(self, config={}):
        # Set default configuration
        self.set_default_config()

        # Update configuration
        for attr, val in config.items():
            setattr(self, attr, val)

        # Calculate properties
        self.init_properties()

    def set_default_config(self):
        self.l = 5 # Vehicle length
        self.s0 = 4 # Vehicle minimum gap
        self.T = 1 # Vehicle time gap
        self.v_max = 16.6 # Vehicle desired speed
        self._v_max = self.v_max
        self.a_max = 1.44 # Vehicle maximum acceleration
        self.b_max = 4.61 # Vehicle comfortable deceleration

        self.path = [] # For recording the paths to be taken
        self.current_road_index = 0
        self.label = 0 # For identifying vehicles
        self.be_front = False # Whether the vehicle is the leading one on the road
        self.total_time = 0

        self.x = 0 # Vehicle's position on the road
        self.v = 0
        self.a = 0
        self.stopped = False

    def init_properties(self):
        self.sqrt_ab = 2*np.sqrt(self.a_max*self.b_max)
#         self._v_max = self.v_max

    def update(self, lead, dt, records):
        # Update position and velocity
        if self.v + self.a*dt < 0:
            self.x -= 1/2*self.v*self.v/self.a
            self.v = 0
        else:
            self.v += self.a*dt
            self.x += self.v*dt + self.a*dt*dt/2

        # Update acceleration
        alpha = 0
        if lead:
            delta_x = lead.x - self.x - lead.l
            delta_v = self.v - lead.v

            alpha = (self.s0 + max(0, self.T*self.v + delta_v*self.v/self.sqrt_ab)) / delta_x

            if delta_x <= 0:
                self.stopped = True
            elif self.x == 0:
                self.stopped = False

        follow_acc = self.a_max * (1-(self.v/self.v_max)**4 - alpha**2)

        if self.stopped:
            stop_acc = -self.b_max*self.v/(self.v_max)
        else:
            stop_acc = follow_acc

        # When a vehicle is following another and also has to stop because of other
        # conditions, the adopted acceleration is the smaller of the two computed
        # above for conservatism
        if self.v > 0:
            self.a = min(follow_acc, stop_acc)
        else:
            self.a = max(min(follow_acc, stop_acc), 0)

        ## Update data of the vehicle in the dataframe of the simulation
        idx = records.loc[records.Vehicle_label==self.label].index.values[0]
        v_label, road_order, total_time, lead_v, stop_time, front_stop_time = list(records.iloc[idx])

        # Initially, when there is no records of the vehicle such as its paths
        # to be taken, the variables need to be set up.
        if len(road_order)==0:
            road_order = self.path # Records all the paths that the vehicle takes
            stop_time = [0]*len(road_order) # Records the amount of time that
                                            # the vehicle stopped on each road
            front_stop_time = [0]*len(road_order) # Records the amount of time
                                                  # that the vehicle stopped on
                                                  # each road while being at the
                                                  # front
        if self.x > 0:
            total_time += dt
            self.total_time += dt

        if self.current_road_index + 1 > len(lead_v): # Check if the vehicle has
                                                      # any next road to enter
            if lead != None:
                # Record the leading vehicle's label
                lead_v.append(lead.label)
            else:
                # When the vehicle itself is the leading vehicle, then use a dummy
                # value to help differentiate
                lead_v.append(-999)

        if self.v == 0: # When the vehicle is stopping
            if lead_v[self.current_road_index] != -999: # The stopped time record
                                                        # is only helpful when the
                                                        # vehicle is not leading
                if self.x != 0:
                    stop_time[self.current_road_index] += dt
            if self.be_front == True:
                front_stop_time[self.current_road_index] += dt

        # Update the record in the dataframe
        records.at[idx, :] = [v_label, road_order, total_time, lead_v, stop_time, front_stop_time]
        return records

    def stop(self):
        self.stopped = True

    def unstop(self):
        self.stopped = False

    def slow(self, v):
        self.v_max = v

    def unslow(self):
        self.v_max = self._v_max
