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
        self.l = 5
        self.s0 = 4
        self.T = 1
        self.v_max = 16.6
        self.a_max = 1.44
        self.b_max = 4.61

        self.path = []
        self.current_road_index = 0
        self.label = 0
        # self.previous_sf = 1

        self.x = 0
        self.v = self.v_max
        self.a = 0
        self.stopped = False

    def init_properties(self):
        self.sqrt_ab = 2*np.sqrt(self.a_max*self.b_max)
        self._v_max = self.v_max

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

        follow_acc = self.a_max * (1-(self.v/self.v_max)**4 - alpha**2)
        # self.a = self.a_max * (1-(self.v/self.v_max)**4 - alpha**2)

        if self.stopped:
            # self.a = -self.b_max*self.v/(self.v_max)
            # print(f"Vehicle {self.label}'s Stopping a is {self.a}")
            stop_acc = -self.b_max*self.v/(self.v_max)

            # stop_acc = -self.b_max*self.v/(self._v_max*self.previous_sf)
        else:
            stop_acc = follow_acc

        self.a = min(follow_acc, stop_acc)
        v_label, road_order, total_time, lead_v, start_stop, gamma, eta = list(records.iloc[self.label])

        if len(road_order)==0:
            road_order = self.path
            start_stop = [0]*len(road_order)
            gamma = self.gamma
            eta = self.eta
        total_time += dt

        if self.current_road_index + 1 > len(lead_v):
            if lead != None:
                lead_v.append(lead.label)
            else:
                lead_v.append(-999)

        # if self.x == 0 and self.v == 0:
        if self.v == 0:
            start_stop[self.current_road_index] += dt

        records.at[self.label, :] = [v_label, road_order, total_time, lead_v, start_stop, gamma, eta]

        # if v_label == 0 and self.current_road_index==1:
        #     print("Leading car's speed is ", self.v)
        # if v_label == 1 and self.current_road_index==0 :
        #     print("Which should match ", lead.v)
        # print(self.current_road_index, self.x, self.v_max, self.v)
        return records

    # def stop(self, previous_sf=1):
    def stop(self):
        # self.previous_sf = previous_sf
        self.stopped = True

    def unstop(self):
        self.stopped = False

    def slow(self, v):
        self.v_max = v

    def unslow(self):
        self.v_max = self._v_max
