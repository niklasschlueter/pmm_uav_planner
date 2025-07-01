#include <pybind11/pybind11.h>

// Forward declare or include your function
int plan_pmm_trajectory(std::string planner_config_file, std::string waypoints_config_file);

PYBIND11_MODULE(pmm_planner, m) {
    m.doc() = "Bindings for PMM trajectory planner";

    m.def("plan_pmm_trajectory", &plan_pmm_trajectory,
          pybind11::arg("planner_config_file"),
          pybind11::arg("waypoints_config_file"),
          "Plan the PMM trajectory and export the result.");
}
