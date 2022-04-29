from scipy.spatial import distance
from collections import deque

class Road:
    def __init__(self, start, end, slow_factor, stop_distance, wait_time):
        self.start = start
        self.end = end
        self.slow_factor = slow_factor # Determine that maximum speed of the road
        self.stop_distance = stop_distance # Determine a part of the road where
                                           # vehicles should start to stop when
                                           # conditions are met
        self.wait_time = 0 # A buffer that controls the minimum time gap between
                           # the timepoints of two consecutive vehicles entering
                           # this road
        self._wait_time = wait_time
        self.vehicles = deque()
        self.driven = False # To differentiate if any vehicles enter the road
        self.appended = False # To differentiate if for a certain frame, any
                              # vehicles have already been added to this road
                              # to avoid double adding

        self.init_properties()

    def init_properties(self):
        self.length = distance.euclidean(self.start, self.end)
        self.angle_sin = (self.end[1]-self.start[1]) / self.length
        self.angle_cos = (self.end[0]-self.start[0]) / self.length
        self.has_traffic_signal = False

    def set_traffic_signal(self, signal, group):
        self.traffic_signal = signal
        self.traffic_signal_group = group
        self.has_traffic_signal = True

    @property
    def traffic_signal_state(self):
        if self.has_traffic_signal:
            i = self.traffic_signal_group
            return self.traffic_signal.current_cycle[i]
        return True

    def update(self, dt, records, front_lead=None):
        n = len(self.vehicles)

        if n > 0:
            # Update first vehicle
            new_records = self.vehicles[0].update(front_lead, dt, records)

            # Update other vehicles
            for i in range(1, n):
                lead = self.vehicles[i-1]
                new_records = self.vehicles[i].update(lead, dt, records)
        try:
            return new_records
        except:
            return records
