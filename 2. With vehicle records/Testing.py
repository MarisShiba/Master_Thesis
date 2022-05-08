import math

def parallel_line(current_coordinates, distance):
    x1, y1 = current_coordinates[0]
    x2, y2 = current_coordinates[1]
    k = (y1-y2)/(x1-x2)
    b = y1 - k*x1

    cross_x = -b/k
    hypo = math.sqrt(b**2+cross_x**2)

    if k>0:
        new_coord_1 = ((x2+distance*abs(b)/hypo, y2-distance*abs(cross_x)/hypo),
                       (x1+distance*abs(b)/hypo, y1-distance*abs(cross_x)/hypo))

        new_coord_2 = ((x2-distance*abs(b)/hypo, y2+distance*abs(cross_x)/hypo),
                       (x1-distance*abs(b)/hypo, y1+distance*abs(cross_x)/hypo))

    else:
        new_coord_1 = ((x2+distance*abs(b)/hypo, y2+distance*abs(cross_x)/hypo),
                       (x1+distance*abs(b)/hypo, y1+distance*abs(cross_x)/hypo))

        new_coord_2 = ((x2-distance*abs(b)/hypo, y2-distance*abs(cross_x)/hypo),
                       (x1-distance*abs(b)/hypo, y1-distance*abs(cross_x)/hypo))

    return new_coord_1, new_coord_2

import numpy as np
from trafficSimulator import *

sim = Simulation()

left, right = -100*math.sqrt(3), 100*math.sqrt(3)
bottom, top = -100, 100

fast_track_factor, slow_track_factor = 1, 0.5
stop_distance = 50
wait_time = 1

left_bottom_outbound = ((left+2, 4), (-5, top-2),
                        slow_track_factor, stop_distance, wait_time)
bottom_right_outbound = ((5, top-2), (right-2, 4),
                         fast_track_factor, stop_distance, wait_time)

left_top_outbound = ((left+2, -4), (-5, bottom+2),
                     fast_track_factor, stop_distance, wait_time)
top_right_outbound = ((5, bottom+2), (right-2, -4),
                      slow_track_factor, stop_distance, wait_time)

bottom_left_inbound = parallel_line(current_coordinates=left_bottom_outbound,
                                    distance=2.5)[0]
right_bottom_inbound = parallel_line(current_coordinates=bottom_right_outbound,
                                     distance=2.5)[1]

top_left_inbound = parallel_line(current_coordinates=left_top_outbound,
                                 distance=2.5)[0]
right_top_inbound = parallel_line(current_coordinates=top_right_outbound,
                                  distance=2.5)[1]

# connection_top_bottom = ((-1.25, bottom+2), (-1.25, top-2))
# connection_bottom_top = ((1.25, top-2), (1.25, bottom+2))

connection_top_bottom = ((-1.25, -2), (-1.25, 2))
connection_bottom_top = ((1.25, 2), (1.25, -2))

sim.create_roads([
    left_bottom_outbound, # Road #0
    bottom_right_outbound, # Road #1

    left_top_outbound, # Road #2
    top_right_outbound, # Road #3

    connection_top_bottom, # Road #4
    connection_bottom_top, # Road #5

    bottom_left_inbound, # Road #6
    right_bottom_inbound, # Road #7

    top_left_inbound, # Road #8
    right_top_inbound, # Road #9

    # Outbound corners
    *curve_road(left_bottom_outbound[1],
                (bottom_right_outbound[0][0], bottom_right_outbound[0][1]+0.01),
                (0, top), 16), # Outbound bottom corner; Roads #10-#25

    *curve_road(left_top_outbound[1],
                (top_right_outbound[0][0], top_right_outbound[0][1]+0.01),
                (0, bottom), 16), # Outbound top corner; Roads #26-#41

    *curve_road(left_bottom_outbound[0],
                (left_top_outbound[0][0]+0.01, left_top_outbound[0][1]),
                (left, 0), 16), # Outbound left corner; Roads #42-#57

    *curve_road(bottom_right_outbound[1],
                (top_right_outbound[1][0]+0.01, top_right_outbound[1][1]),
                (right, 0), 16), # Outbound right corner; Roads #58-#73

    # Inbound corners
    *curve_road(right_bottom_inbound[1],
                (bottom_left_inbound[0][0], bottom_left_inbound[0][1]+0.01),
                (0, top), 16), # Inbound bottom corner; Roads #74-#89

    *curve_road(right_top_inbound[1],
                (top_left_inbound[0][0], top_left_inbound[0][1]+0.01),
                (0, bottom), 16), # Inbound top corner; Roads #90-#105

    *curve_road(bottom_left_inbound[1],
                (top_left_inbound[1][0]+0.01, top_left_inbound[1][1]),
                (left+2.5, 0), 16), # Inbound left corner; Roads #106-#121

    *curve_road(right_bottom_inbound[0],
                (right_top_inbound[0][0]+0.01, right_top_inbound[0][1]),
                (right-2.5, 0), 16), # Inbound right corner; Roads #122-#137
])


sim.create_gen({
    'vehicle_rate': 60,
    'vehicles': [
#         [1, {'path': [2, *range(26, 34), 4, *range(18, 26), 1]}],
        [1, {'path': [0, 3], 'v_max': 15, 's0': 4, 'T': 1, 'b_max': 3, 'a_max': 1}],
        [1, {'path': [0, 1], 'v_max': 15, 's0': 4, 'T': 1, 'b_max': 3, 'a_max': 1}],

        [1, {'path': [2, 3], 'v_max': 15, 's0': 4, 'T': 1, 'b_max': 5, 'a_max': 1}],
        [1, {'path': [2, 1], 'v_max': 15, 's0': 4, 'T': 1, 'b_max': 5, 'a_max': 1}],


#         [1, {'path': [0, *range(10, 18), 1]}],
#         [1, {'path': [1]}]
    ],
    'vehicle_limit': 100
    })


# Start simulation
win = Window(sim)
win.zoom = 3
win.run(steps_per_update=5)
