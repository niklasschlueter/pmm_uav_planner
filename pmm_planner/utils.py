import pmm_planner
import yaml
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# Somehwat useful for debugging segfaults
# import faulthandler
# faulthandler.enable()


def plan_pmm_trajectory_from_waypoints_file(
    waypoints_config_file: str,
    planner_config_file: str,
    sampling_period: float = 0.05,
    csv_path: str | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Plans a PMM trajectory using a waypoints configuration file and a planner configuration file.

    This function loads the waypoints from a YAML file and calls `plan_pmm_trajectory`
    with the loaded configuration.

    Args:
        waypoints_config_file (str): Path to the YAML file containing waypoints configuration.
        planner_config_file (str): Path to the YAML file containing planner parameters.
        sampling_period (float, optional): Time interval for sampling the trajectory. Defaults to 0.05.
        csv_path (str, optional): If provided, the sampled trajectory will be exported to this CSV file path.

    Returns:
        tuple: A tuple containing:
            - t_s (np.ndarray): Sampled timestamps.
            - p_s (np.ndarray): Sampled positions.
            - v_s (np.ndarray): Sampled velocities.
            - a_s (np.ndarray): Sampled accelerations.
    """
    # Load YAML
    with open(waypoints_config_file) as f:
        waypoints_config = yaml.safe_load(f)

    return plan_pmm_trajectory(
        waypoints_config, planner_config_file, sampling_period
    )  # , csv_path)


def plan_pmm_trajectory(
    waypoints_config: dict,
    planner_config_file: str = str(
        Path(__file__).parent.parent / "config" / "planner" / "crazyflie.yaml"
    ),
    sampling_period: float = 0.05,
    # csv_path: str | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Plans a PMM trajectory given waypoints and planner configuration.

    Loads planner parameters from file, constructs a PMM trajectory, samples it,
    optionally exports it to a CSV file, and plots the resulting trajectory.

    Args:
        waypoints_config (dict): Dictionary containing start/end velocities and waypoints.
        planner_config_file (str): Path to the YAML file containing planner parameters.
        sampling_period (float, optional): Time interval for sampling the trajectory. Defaults to 0.05.
        csv_path (str, optional): If provided, the sampled trajectory will be exported to this CSV file path.

    Returns:
        tuple: A tuple containing:
            - t_s (np.ndarray): Sampled timestamps.
            - p_s (np.ndarray): Sampled positions.
            - v_s (np.ndarray): Sampled velocities.
            - a_s (np.ndarray): Sampled accelerations.
    """

    with open(planner_config_file) as f:
        planner_config = yaml.safe_load(f)

    # Extract stuff — same as C++
    max_acc_norm = planner_config["uav"]["maximum_acceleration_norm"]
    max_vel_norm = planner_config["uav"]["maximum_velocity"]
    dT_precision = planner_config["velocity_optimization"]["dT_precision"]
    max_iter = planner_config["velocity_optimization"]["first_run"]["max_iter"]
    alpha = planner_config["velocity_optimization"]["first_run"]["alpha"]
    alpha_red_factor = planner_config["velocity_optimization"]["first_run"][
        "alpha_reduction_factor"
    ]
    alpha_min_thr = planner_config["velocity_optimization"]["first_run"]["alpha_min_threshold"]

    TD_max_iter = planner_config["thrust_decomposition"]["max_iter"]
    TD_acc_precision = planner_config["thrust_decomposition"]["precision"]
    drag = planner_config["thrust_decomposition"]["drag"]

    max_iter2 = planner_config["velocity_optimization"]["second_run"]["max_iter"]
    alpha2 = planner_config["velocity_optimization"]["second_run"]["alpha"]
    alpha_red_factor2 = planner_config["velocity_optimization"]["second_run"][
        "alpha_reduction_factor"
    ]
    alpha_min_thr2 = planner_config["velocity_optimization"]["second_run"]["alpha_min_threshold"]

    debug = planner_config["debug"]

    # Previous definition required this
    # start_pos = waypoints_config["start"]["position"]
    # end_pos = waypoints_config["end"]["position"]
    # waypoints = [start_pos] + waypoints + [end_pos]

    # Evaluate waypoints_config
    start_vel = waypoints_config["start_velocity"]
    end_vel = waypoints_config["end_velocity"]
    waypoints = waypoints_config["waypoints"]

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
    return traj
