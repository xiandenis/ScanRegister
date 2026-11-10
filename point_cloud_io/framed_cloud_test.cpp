#include "framed_cloud.hpp"
#include "point_cloud_io.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>

namespace {

int g_fails = 0;

void expect(bool cond, char const* msg)
{
  if (!cond) {
    std::cerr << "FAIL: " << msg << "\n";
    ++g_fails;
  }
}

bool near_eq(double a, double b, double eps = 1e-5)
{
  return std::abs(a - b) <= eps;
}

}  // namespace

int main()
{
  using Ddx::point_cloud_io::FramedCloud;
  using Ddx::point_cloud_io::is_valid_origin_candidate;
  using Ddx::point_cloud_io::localize_world_cloud;

  expect(!is_valid_origin_candidate(0.0, 0.0, 0.0), "reject exact zero");
  expect(!is_valid_origin_candidate(std::numeric_limits<double>::quiet_NaN(), 1.0, 1.0),
         "reject NaN");
  expect(!is_valid_origin_candidate(1.0, 1.0, std::numeric_limits<double>::infinity()),
         "reject Inf");
  expect(is_valid_origin_candidate(1e6, 2e6, 100.0), "accept large finite");

  {
    FramedCloud<pcl::PointXYZ> framed;
    framed.cloud.push_back(pcl::PointXYZ(0.f, 0.f, 0.f));
    framed.cloud.push_back(pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(), 0.f, 0.f));
    framed.cloud.push_back(pcl::PointXYZ(1.0e6f, 2.0e6f, 100.f));
    framed.cloud.push_back(pcl::PointXYZ(1.0e6f + 10.f, 2.0e6f + 5.f, 101.f));
    expect(localize_world_cloud(framed) == 0, "localize succeeds");
    expect(near_eq(framed.origin.x(), 1.0e6, 1.0) && near_eq(framed.origin.y(), 2.0e6, 1.0) &&
               near_eq(framed.origin.z(), 100.0, 1e-3),
           "origin is first valid world point");
    expect(near_eq(framed.cloud[2].x, 0.0, 1e-3) && near_eq(framed.cloud[2].y, 0.0, 1e-3) &&
               near_eq(framed.cloud[2].z, 0.0, 1e-3),
           "anchor point local near zero");
    expect(near_eq(framed.cloud[3].x, 10.0, 1e-2) && near_eq(framed.cloud[3].y, 5.0, 1e-2) &&
               near_eq(framed.cloud[3].z, 1.0, 1e-2),
           "relative geometry preserved");
  }

  {
    FramedCloud<pcl::PointXYZ> framed;
    framed.cloud.push_back(pcl::PointXYZ(0.f, 0.f, 0.f));
    framed.cloud.push_back(pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(), 0.f, 0.f));
    expect(localize_world_cloud(framed) != 0, "all-invalid fails");
    expect(framed.cloud.empty(), "cloud cleared on failure");
  }

  {
    using Ddx::point_cloud_io::save_las;
    using Ddx::point_cloud_io::load_las;
    FramedCloud<pcl::PointXYZ> out;
    out.origin = Eigen::Vector3d(1234567.0, 7654321.0, 100.0);
    out.cloud.push_back(pcl::PointXYZ(0.f, 0.f, 0.f));
    out.cloud.push_back(pcl::PointXYZ(10.f, 5.f, 1.f));
    out.cloud.push_back(pcl::PointXYZ(-3.f, 2.f, 0.5f));
    const auto path =
        (std::filesystem::temp_directory_path() / "framed_cloud_roundtrip.las").string();
    expect(save_las(path, out) == 0, "save_las ok");
    FramedCloud<pcl::PointXYZ> back;
    expect(load_las(path, back) == 0, "load_las ok");
    expect(back.cloud.size() == 3, "roundtrip point count");
    // Relative geometry: first valid world point becomes new origin → first local ≈ 0.
    expect(near_eq(back.cloud[0].x, 0.0, 1e-2) && near_eq(back.cloud[0].y, 0.0, 1e-2) &&
               near_eq(back.cloud[0].z, 0.0, 1e-2),
           "roundtrip first local near zero");
    expect(near_eq(back.cloud[1].x - back.cloud[0].x, 10.0, 1e-2) &&
               near_eq(back.cloud[1].y - back.cloud[0].y, 5.0, 1e-2) &&
               near_eq(back.cloud[1].z - back.cloud[0].z, 1.0, 1e-2),
           "roundtrip relative offsets");
    std::error_code ec;
    std::filesystem::remove(path, ec);
  }

  if (g_fails != 0) {
    std::cerr << g_fails << " failure(s)\n";
    return 1;
  }
  std::cout << "framed_cloud_test: OK\n";
  return 0;
}
