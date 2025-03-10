import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np

def decompose_X(X, state_dim, total_dim):
    x_new = X
    qs = []
    q_dots = []
    us = []
    for i in range(state_dim):
        qs.append(x_new[i+1::total_dim])
        q_dots.append(x_new[i+1+state_dim::total_dim])
        us.append(x_new[i+1+2*state_dim::total_dim])
    return np.array(qs), np.array(q_dots), np.array(us)

def plot_3d_trajectory(Xn, points, state_dim = 3, qs_size = 9,  show=True, title='3D Trajectory', cur_label='trajectory'):
    ''' Plot the 3D trajectory given the points to visit, the shortest path and the dictionary of results from the shortest path'''

    fig = plt.figure(figsize=(15, 10))  
    ax = fig.add_subplot(111, projection='3d')
    tf = float(Xn[0])
    qs, qs_dots, us = decompose_X(Xn, state_dim, qs_size)

 
    # generate trajectory graph
    qs = [k for k in zip(*qs)]
    qs_dots = [k for k in zip(*qs_dots)]
    ee_list = []
    vel_ee_mag = []
    for j in range(len(qs)):
        ee_list.append(qs[j])
        vel_ee = qs_dots[j]
        vel_ee_mag.append(np.linalg.norm(vel_ee))
    # end effector path
    x = [float(point[0]) for point in ee_list]
    y = [float(point[1]) for point in ee_list]
    z = [float(point[2]) for point in ee_list]
    
    # plot end effector path
    ax.plot(x, y, z, label=cur_label)

    # plot points
    ax.scatter([point[0] for point in points[1:]], [point[1] for point in points[1:]], [point[2] for point in points[1:]], color='green', label='Points to visit')
    # plot start point
    ax.scatter(points[0][0], points[0][1], points[0][2], color='purple', label='Start point')
    ax.legend()
    # add title
    ax.set_title(title)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    if not show:
        return fig
    plt.show()


def compare_trajectories_casadi_plot(Xn, points, state_dim = 3, qs_size = 9, show=True, v_bounds=None, a_bounds=None):
    # state dim, ts and Ns are lists
    cur_label = 'Trajectory'
    # plot X
    number_of_plots = 3 + 2*state_dim
    # define the subplots
    rows = int(np.ceil(np.sqrt(number_of_plots)))
    cols = int(np.ceil(number_of_plots / rows))
    fig, axes = plt.subplots(rows, cols, figsize=(15, 15))
    axes = np.ravel(axes)


    tf = float(Xn[0])   
    qs, qs_dots, us = decompose_X(Xn, state_dim, qs_size)

    # generate trajectory graph
    qs = [k for k in zip(*qs)]
    ee_list = []
    for j in range(len(qs)):
        ee_list.append(qs[j])

    # plot x-y trajectory
    x = [float(point[0]) for point in ee_list]
    y = [float(point[1]) for point in ee_list]
    axes[0].plot(x, y, label=f'{cur_label}')
    axes[0].set_title('x - y trajectories')
    axes[0].set_xlabel('x')
    axes[0].set_ylabel('y')
    # plot points
    axes[0].scatter([point[0] for point in points[1:]], [point[1] for point in points[1:]], color='green', label='Points to visit')
    # plot start point
    axes[0].scatter(points[0][0], points[0][1], color='purple', label='Start point')

    # plot x-z trajectory
    x = [float(point[0]) for point in ee_list]
    z = [float(point[2]) for point in ee_list]
    axes[1].plot(x, z, label=f'{cur_label}')
    axes[1].set_title('x - z trajectories')
    axes[1].set_xlabel('x')
    axes[1].set_ylabel('z')
    axes[1].scatter([point[0] for point in points[1:]], [point[2] for point in points[1:]], color='green', label='Points to visit')
    axes[1].scatter(points[0][0], points[0][2], color='purple', label='Start point')

    # plot y-z trajectory
    axes[2].plot(y, z, label=f'{cur_label}')
    axes[2].set_title('y - z trajectories')
    axes[2].set_xlabel('y')
    axes[2].set_ylabel('z')
    axes[2].scatter([point[1] for point in points[1:]], [point[2] for point in points[1:]], color='green', label='Points to visit')
    axes[2].scatter(points[0][1], points[0][2], color='purple', label='Start point')
    
    # plot velocities and accelerations
    t = np.linspace(0, tf, len(qs_dots[0]))
    for j in range(state_dim):
        v = [float(qs_dots[j][k]) for k in range(len(qs_dots[j]))]
        axes[j+3].plot(t, v, label=f'{cur_label}')
        if v_bounds:
            axes[j+3].plot(t, [v_bounds[0][j]]*len(t), 'r--')
            axes[j+3].plot(t, [v_bounds[1][j]]*len(t), 'r--')
        axes[j+3].set_title(f'Velocities of q{j}')
        axes[j+3].set_xlabel('Time')
        axes[j+3].set_ylabel('Velocity')
    # plot accelerations
    for j in range(state_dim):
        a = [float(us[j][k]) for k in range(len(us[j]))]
        if a_bounds:
            axes[j+state_dim+1].plot(t, [a_bounds[i][0][j]]*len(t), 'r--')
            axes[j+state_dim+1].plot(t, [a_bounds[i][1][j]]*len(t), 'r--')
        axes[j+state_dim+3].plot(t, a, label=f'{cur_label}')
        axes[j+state_dim+3].set_title(f'Accelerations of q{j}')
        axes[j+state_dim+3].set_xlabel('Time')
        axes[j+state_dim+3].set_ylabel('Acceleration')

    fig.subplots_adjust(hspace=0.4, wspace=0.4)
    if not show:
        return fig
    plt.show()