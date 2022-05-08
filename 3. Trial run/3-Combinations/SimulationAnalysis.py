import numpy as np
import pandas as pd

def utility_summary(all_routes, record_df):
    """
    Returns a pivot table that summarizes the utility related features of each vehicle.

    Input:
        all_routes (list): all possible routes for vehicles to take
        record_df (pandas DataFrame): records the simulation data of each vehicle

    """

    utility_tmp = record_df[['Vehicle_label', 'Road_order', 'Total_time', 'Round_number']]

    utility_tmp['Caused_delay'] = 0.0
    utility_tmp.Total_time = pd.to_numeric(utility_tmp.Total_time)

    for idx, row in record_df.iterrows():
        current_l = row['Vehicle_label']
        stop_times = row['Stopped_time']
        lead_v = row['Leading_vehicles']
        route = row['Road_order']
        round_num = row['Round_number']

        # For each vehicle's stop time, find its leading vehicle of the corresponding road segment
        # and assign the stop time to that vehicle as the time of delay that it causes
        for i in range(len(stop_times)):
            if stop_times[i] > 0:
                utility_tmp.loc[(utility_tmp.Vehicle_label==lead_v[i]) &\
                                (utility_tmp.Round_number==round_num), 'Caused_delay'] += stop_times[i]

        # Record the label of the routes taken
        for r in all_routes:
            if route == r:
                utility_tmp.at[idx, 'Road_order'] = all_routes.index(r)

    utility_tmp['Count'] = 1
    utility_tmp = pd.pivot_table(utility_tmp,
                                 index=['Vehicle_label', 'Road_order'],
                                 values=['Total_time', 'Caused_delay', 'Count'],
                                 aggfunc={'Total_time': np.mean, # Average time taken for the past rounds
                                          'Caused_delay': np.mean, # Average delay caused for the
                                                                   # past rounds
                                          'Count': 'count'} # Count how many times the vehicle has taken
                                                            # the same routes
                                ).reset_index(drop=False)
    return utility_tmp

def compute_utility(explored_times,
                    time=0,
                    caused_delay=0,
                    alpha=0,
                    gamma=0,
                    eta=0,
                    delta=0.1):
    """
    Returns the computed utility value

    Input:
        explored_times (integer): the number of times that the vehicle has taken the
                                  same route
        time (float): the amount of time taken to reach the destination
        caused_delay (float): the amount of delay that the vehicle has caused
        alpha (float): the penalty level
        gamma (float): the interpolation between money and time spent
        eta (float): the level of risk aversion
        delta (float): a parameter of the Upper Confidence Bound equation that controls
                       extent of encouraging exploration

    """

    if explored_times > 0:

        if alpha != 0 and caused_delay != 0:
            u_penalty = -np.log(caused_delay*alpha)
        else:
            u_penalty = 0

        if eta == 1:
            u_time = -np.log(gamma*time)
        elif time!=0:
            u_time = ((gamma*time)**(eta-1)-1) / (1-eta)
        else:
            u_time = 0

        u_total = u_time + u_penalty
    else:
        u_total = 0

    if explored_times < 5:
        u_total += np.sqrt(2*np.log(1/delta)/explored_times)

    return u_total

def compute_probability(utility_list):
    """
    Returns a list of probabilities of taking each route (using softmax)

    """
    prob_list = [np.exp(u) for u in utility_list]
    return [p/sum(prob_list) for p in prob_list]

def update_utility_df(all_routes, delta, alpha, record_df, df, utility_tmp, round_number=1):
    """
    Returns an updated dataframe that stores utility data of all vehicles

    """

    utility_df = df.copy()

    for idx, row in utility_df.iterrows():
        i = row['Vehicle_label']
        utility_l = row['Utilities'][:]
        prob_l = row['Probabilities'][:]
        routes = row['Routes_taken'][:]
        gamma = row['Gamma']
        eta = row['Eta']

        for j in range(len(all_routes)):
            record = utility_tmp.loc[(utility_tmp.Vehicle_label==i) & (utility_tmp.Road_order==j)]
            if len(record) == 0:
                utility_l[j] = compute_utility(explored_times=0.01, delta=delta) # Exploration time is not integer here
                                                                                 # because utility will be infinite if
                                                                                 # it is 0 and it will cause problems in prob
                                                                                 # calculation
            else:
                utility_l[j] = compute_utility(explored_times=record['Count'].values[0],
                                               time=record['Total_time'].values[0],
                                               caused_delay=record['Caused_delay'].values[0],
                                               alpha=alpha,
                                               gamma=gamma,
                                               eta=eta,
                                               delta=delta)

        # Update probabilities of taking each route
        prob_l = compute_probability(utility_l)

        # Append the route chosen for the last round
        new_route_list = record_df.loc[(record_df.Vehicle_label==i) &
                                       (record_df.Round_number==round_number)]['Road_order'].values[0]
        new_route = all_routes.index(new_route_list)
        routes.append(new_route)

        # Update dataframe
        utility_df.at[idx, 'Utilities'] = utility_l
        utility_df.at[idx, 'Probabilities'] = prob_l
        utility_df.at[idx, 'Routes_taken'] = routes

    return utility_df
