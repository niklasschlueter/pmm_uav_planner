import sys

#sys.path.append("/home/niklas/repos/pmm_uav_planner_custom/build")
#sys.path.append("/home/niklas/repos/pmm_uav_planner/build")
import pmm_planner
import yaml
import numpy as np
import pmm_planner
import matplotlib.pyplot as plt
import time

import faulthandler

faulthandler.enable()


def plan_pmm_trajectory(
    planner_config_file, waypoints_config_file, sampling_period=0.05, csv_path=None
):
    # Load YAML
    with open(planner_config_file) as f:
        planner_config = yaml.safe_load(f)

    with open(waypoints_config_file) as f:
        waypoints_config = yaml.safe_load(f)

    print(f"planner config: {planner_config}")
    print(f"waypoints config: {waypoints_config}")

    # Extract stuff — same as C++
    max_acc_norm = planner_config["uav"]["maximum_acceleration_norm"]
    max_vel_norm = planner_config["uav"]["maximum_velocity"]
    dT_precision = planner_config["velocity_optimization"]["dT_precision"]
    max_iter = planner_config["velocity_optimization"]["first_run"]["max_iter"]
    alpha = planner_config["velocity_optimization"]["first_run"]["alpha"]
    alpha_red_factor = planner_config["velocity_optimization"]["first_run"][
        "alpha_reduction_factor"
    ]
    alpha_min_thr = planner_config["velocity_optimization"]["first_run"][
        "alpha_min_threshold"
    ]

    TD_max_iter = planner_config["thrust_decomposition"]["max_iter"]
    TD_acc_precision = planner_config["thrust_decomposition"]["precision"]
    drag = planner_config["thrust_decomposition"]["drag"]

    max_iter2 = planner_config["velocity_optimization"]["second_run"]["max_iter"]
    alpha2 = planner_config["velocity_optimization"]["second_run"]["alpha"]
    alpha_red_factor2 = planner_config["velocity_optimization"]["second_run"][
        "alpha_reduction_factor"
    ]
    alpha_min_thr2 = planner_config["velocity_optimization"]["second_run"][
        "alpha_min_threshold"
    ]

    debug = planner_config["debug"]

    start_pos = waypoints_config["start"]["position"]
    end_pos = waypoints_config["end"]["position"]
    start_vel = waypoints_config["start"]["velocity"]
    end_vel = waypoints_config["end"]["velocity"]
    waypoints = waypoints_config["waypoints"]
    waypoints = [start_pos] + waypoints + [end_pos]

    print(f"before construction")
    print(f"waypoints: {waypoints}")
    # waypoints = np.array(waypoints)

    # waypoints = [np.array(wp, dtype=np.float64) for wp in waypoints]
    # Construct
    traj = pmm_planner.PMM_MG_Trajectory3D(
        waypoints,
        start_vel,
        end_vel,
        max_acc_norm,
        max_vel_norm,
        dT_precision,
        max_iter,
        alpha,
        alpha_red_factor,
        alpha_min_thr,
        TD_max_iter,
        TD_acc_precision,
        True,  # second_round_opt
        max_iter2,
        alpha2,
        alpha_red_factor2,
        alpha_min_thr2,
        drag,
        debug,
    )

    print("Trajectory duration:", traj.duration())

    t_s, p_s, v_s, a_s = traj.get_sampled_trajectory(sampling_period)
    t_s, p_s, v_s, a_s = np.array(t_s), np.array(p_s), np.array(v_s), np.array(a_s)

    # print("Times:", t_s[:])
    # print("First pos:", p_s[:])
    # print("First vel:", v_s[:])
    # print("First acc:", a_s[:])

    if csv_path is not None:
        traj.sample_and_export_trajectory(sampling_period, csv_path)

    ax = plt.figure().add_subplot(projection="3d")
    print(f"shape ps: {np.shape(p_s), type(p_s)}")
    ax.plot(p_s[:, 0], p_s[:, 1], p_s[:, 2])
    plt.show()


if __name__ == "__main__":
    t0 = time.perf_counter()
    planner_config_file = "../config/planner/planner_config.yaml"
    waypoints_config_file = "../config/waypoints/eight.yaml"
    plan_pmm_trajectory(planner_config_file, waypoints_config_file, csv_path=None)
    t1 = time.perf_counter()
    print(f"time to plan: {t1 - t0}")
