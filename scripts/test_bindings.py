import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from pmm_planner.utils import plan_pmm_trajectory_from_waypoints_file
from pmm_planner.utils import plan_pmm_trajectory


def debug_plot(p_s):
    ax = plt.figure().add_subplot(projection="3d")
    ax.plot(p_s[:, 0], p_s[:, 1], p_s[:, 2])
    plt.show()


if __name__ == "__main__":
    # Set sources - only works if run form scripts directory
    planner_config_file = "../config/planner/crazyflie.yaml"
    waypoints_config_file = "../config/waypoints/hypotrochoid.yaml"

    # with waypoints file
    traj = plan_pmm_trajectory_from_waypoints_file(waypoints_config_file, planner_config_file)

    sampling_period = 0.05
    t_s, p_s, v_s, a_s = traj.get_sampled_trajectory(sampling_period)
    t_s, p_s, v_s, a_s = np.array(t_s), np.array(p_s), np.array(v_s), np.array(a_s)

    print(f"duration: {traj.duration()}")

    # if csv_path is not None:
    # traj.sample_and_export_trajectory(sampling_period, csv_path)

    debug_plot(p_s)
    waypoints_config = {
        "start_velocity": [0, 0, 0],
        "end_velocity": [0, 0, 0],
        "waypoints": np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0]]),
    }
    # with waypoints dict
    traj = plan_pmm_trajectory(waypoints_config, planner_config_file)

    sampling_period = 0.05
    t_s, p_s, v_s, a_s = traj.get_sampled_trajectory(sampling_period)
    t_s, p_s, v_s, a_s = np.array(t_s), np.array(p_s), np.array(v_s), np.array(a_s)
    debug_plot(p_s)
