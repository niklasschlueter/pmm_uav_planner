#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "pmm_mg_trajectory3d.hpp"

namespace py = pybind11;

PYBIND11_MODULE(pmm_planner, m) {
    py::class_<pmm::PMM_MG_Trajectory3D>(m, "PMM_MG_Trajectory3D")
        .def(py::init<
            std::vector<pmm::Vector<3>>,
            const pmm::Vector<3>,
            const pmm::Vector<3>,
            const pmm::Scalar,
            const pmm::Scalar,
            const pmm::Scalar,
            const int,
            const pmm::Scalar,
            const pmm::Scalar,
            const pmm::Scalar,
            const int,
            const pmm::Scalar,
            const bool,
            const int,
            const pmm::Scalar,
            const pmm::Scalar,
            const pmm::Scalar,
            const bool,
            const bool
        >())

        .def("duration", &pmm::PMM_MG_Trajectory3D::duration)
        .def("sample_and_export_trajectory", &pmm::PMM_MG_Trajectory3D::sample_and_export_trajectory)
	.def("get_sampled_trajectory", &pmm::PMM_MG_Trajectory3D::get_sampled_trajectory)
        ;
}
