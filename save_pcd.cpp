#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace py = pybind11;

struct PointXYZIT {
    PCL_ADD_POINT4D;                  // quad-word XYZ
    float intensity;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// Register custom point type
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
)

bool save_pcd(const std::string &filename, const std::vector<std::vector<float>> &points, double timestamp) {
    pcl::PointCloud<PointXYZIT> cloud;
    for (const auto& pt : points) {
        if (pt.size() >= 4) {
            PointXYZIT p;
            p.x = pt[0];
            p.y = pt[1];
            p.z = pt[2];
            p.intensity = pt[3];
            p.timestamp = timestamp;
            cloud.push_back(p);
        }
    }
    return pcl::io::savePCDFileBinary(filename, cloud) == 0;
}

PYBIND11_MODULE(save_pcd_module, m) {
    m.def("save_pcd", &save_pcd, "Save XYZ + intensity + timestamp to .pcd",
          py::arg("filename"), py::arg("points"), py::arg("timestamp"));
}

