#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration_helper.hpp>

#include <Eigen/Core>
#include <vector>

int main() {
  std::vector<Eigen::Vector3d> target{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 0},
                                      {1, 0, 1}, {0, 1, 1}, {1, 1, 1}, {0.5, 0.5, 0}, {0.5, 0, 0.5}};
  std::vector<Eigen::Vector3d> source = target;
  for (auto& p : source) {
    p.x() += 0.05;
  }

  small_gicp::RegistrationSetting setting;
  setting.type = small_gicp::RegistrationSetting::GICP;
  setting.num_threads = 1;
  setting.downsampling_resolution = 0.5;
  setting.max_correspondence_distance = 1.0;

  const auto result = small_gicp::align(target, source, Eigen::Isometry3d::Identity(), setting);
  return result.converged ? 0 : 1;
}
