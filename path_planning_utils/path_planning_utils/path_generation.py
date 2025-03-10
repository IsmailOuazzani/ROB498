import numpy as np  
import yaml
import os

from path_planning_utils.point_mass_trajectory_optimization import space_curve, velocity_curve, acceleration_curve
from path_planning_utils.trajectory_computations import compute_all_trajectories
from path_planning_utils.velocity_generation import compute_velocities_in_cone_3d
from path_planning_utils.plotting import plot_3d_trajectory, compare_trajectories_casadi_plot

def load_params(file_path):
    with open(file_path, "r") as file:
        params = yaml.safe_load(file)
    return params

def initial_guess(points,  position_noise=0, velocity_noise=0, acceleration_noise=0, config=os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "planning_params.yaml")):
    # load params
    params = load_params(config)
    max_vel = [params['max_vel']['x'], params['max_vel']['y'], params['max_vel']['z']]
    min_vel = [params['min_vel']['x'], params['min_vel']['y'], params['min_vel']['z']]
    max_acc = [params['max_acc']['x'], params['max_acc']['y'], params['max_acc']['z']]
    min_acc = [params['min_acc']['x'], params['min_acc']['y'], params['min_acc']['z']]

    magnitude_step = params['magnitude_step']
    angle_step = params['angle_step']
    dt = params['dt']
    vertex_angle_deg = params['vertex_angle_deg']
    
    max_vel_mag = np.linalg.norm(max_vel)
    velocity_df = compute_velocities_in_cone_3d(points, 0, max_vel_mag, vertex_angle_deg=vertex_angle_deg, magnitude_step=magnitude_step, angle_step=angle_step, loop=True, prediction_horizon=len(points))
    # compute velocity df
    results, dict_res, shortest = compute_all_trajectories(points, velocity_df, max_acc, min_acc, max_vel, min_vel, prediction_horizon=None)
    tf = compute_tf(shortest, dict_res)
    # calculate N based on dt
    N = int(tf/dt)
    Ns, tf = compute_Ns(shortest, dict_res, N)
    ts = []
    for i in range(len(shortest)-1):
        t = dict_res[shortest[i]][shortest[i+1]][0][6]
        ts.append(t)
    X0_no_tn = []   

    for i in range(len(shortest)-1):
        # qs, q_dots, us = equally_spaced_in_place(points, i, shortest, dict_res, Ns[i])
        qs, q_dots, us = equally_spaced_in_time(i, shortest, dict_res, Ns[i])
        qs_reshaped = np.array(qs)
        q_dots_reshaped = np.array(q_dots)
        us_reshaped = np.array(us)
        for k in range(Ns[i]): 
            if k == 0 and i == 0:
                X0_no_tn.extend([*qs_reshaped[:,k], *q_dots_reshaped[:,k], *us_reshaped[:,k]])
            else:
                X0_no_tn.extend([*qs_reshaped[:,k]+position_noise*np.random.random(qs_reshaped[:,k].shape), *q_dots_reshaped[:,k]+velocity_noise*np.random.random(q_dots_reshaped[:,k].shape), *us_reshaped[:,k]+acceleration_noise*np.random.random(us_reshaped[:,k].shape)])
                                                            
    return X0_no_tn, tf, N


def equally_spaced_in_time(i, shortest, dict_res, N=100):
    params = dict_res[shortest[i]][shortest[i+1]]
    if i ==0:
        t_values = np.linspace(0, params[0][6], N)
    else:
        t_values = np.linspace(0, params[0][6], N+1) # this way, there will not be two overlapping points
        t_values = t_values[1:]

    qs = []
    q_dots = []
    us = []
    for i in range(len(params)):
        qs.append([space_curve(t, params[i][0], params[i][1], params[i][2], params[i][3], params[i][4], params[i][5], params[i][7], params[i][8]) for t in t_values])
        q_dots.append([velocity_curve(t, params[i][1], params[i][2], params[i][3], params[i][4], params[i][5], params[i][7], params[i][8]) for t in t_values])
        us.append([acceleration_curve(t, params[i][2], params[i][5], params[i][7], params[i][8]) for t in t_values])
    return qs, q_dots, us

def equally_spaced_in_place(points, i, shortest, dict_res, N=100):
    qs_bad, q_dots, us = equally_spaced_in_time(i, shortest, dict_res, N)

    qs = []
    params = dict_res[shortest[i]][shortest[i+1]]
    for i in range(len(qs_bad)):
        starting_point = space_curve(0, params[i][0], params[i][1], params[i][2], params[i][3], params[i][4], params[i][5], params[i][7], params[i][8])
        ending_point = space_curve(params[i][6], params[i][0], params[i][1], params[i][2], params[i][3], params[i][4], params[i][5], params[i][7], params[i][8])
        qs.append(np.linspace(starting_point, ending_point, N))

    return qs, q_dots, us

def compute_tf(shortest, dict_res):
    tf = 0
    for i in range(len(shortest)-1):
        tf += dict_res[shortest[i]][shortest[i+1]][0][6]
    return tf

def compute_Ns(shortest, dict_res, N):
    Ns = []

    tf = compute_tf(shortest, dict_res)
    ts = np.linspace(0, tf, N)
    t_pref = 0
    for i in range(len(shortest)-1):
        t = dict_res[shortest[i]][shortest[i+1]][0][6]
        # find how many points to sample from t_pref to t
        N_to_append = int(N*(t-t_pref)/tf)
        if i == len(shortest)-2:
            N_to_append = N - sum(Ns)
        Ns.append(N_to_append)
    return Ns, tf 


if __name__ == '__main__':
    points = [[0, 0, 0], [1, 1, 4], [2, -3, 2   ], [3, 3, 3]]
    X0_no_tn, tf, N = initial_guess(points)
    # combine the results
    X0 = [tf]
    X0.extend(X0_no_tn)
    # plot_3d_trajectory(X0, points, show=True)
    print(f"tf: {tf}")
    compare_trajectories_casadi_plot(X0, points, show=True)


