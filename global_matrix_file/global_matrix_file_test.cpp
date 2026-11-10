#include <global_matrix_file/global_matrix_file_io.hpp>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>

namespace {

int g_failures = 0;

void check(char const* name, bool ok)
{
  std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << '\n';
  if (!ok) {
    ++g_failures;
  }
}

Eigen::Matrix4d makePose(double x, double y, double z, double yaw_rad)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 0) = std::cos(yaw_rad);
  T(0, 1) = -std::sin(yaw_rad);
  T(1, 0) = std::sin(yaw_rad);
  T(1, 1) = std::cos(yaw_rad);
  T(0, 3) = x;
  T(1, 3) = y;
  T(2, 3) = z;
  return T;
}

bool nearMat(Eigen::Matrix4d const& a, Eigen::Matrix4d const& b, double eps = 1e-6)
{
  return (a - b).cwiseAbs().maxCoeff() <= eps;
}

}  // namespace

int main()
{
  std::cout << "=== global_matrix_file unit smoke ===\n\n";

  std::map<std::string, Eigen::Matrix4d> poses;
  poses.emplace("A", makePose(1.0, 2.0, 3.0, 0.1));
  poses.emplace("B", makePose(-4.0, 5.5, 0.25, -0.35));

  auto const path =
      (std::filesystem::temp_directory_path() / "scan_register_global_matrix_smoke.json").string();

  check("saving", Ddx::GlobalMatrixFileIO::saving(path, poses));

  std::map<std::string, Eigen::Matrix4d> loaded;
  check("loading", Ddx::GlobalMatrixFileIO::loading(path, loaded));
  check("station count", loaded.size() == 2);
  check("has A", loaded.count("A") == 1);
  check("has B", loaded.count("B") == 1);
  if (loaded.count("A") && loaded.count("B")) {
    check("pose A roundtrip", nearMat(loaded.at("A"), poses.at("A")));
    check("pose B roundtrip", nearMat(loaded.at("B"), poses.at("B")));
  }

  std::error_code ec;
  std::filesystem::remove(path, ec);

  std::cout << '\n';
  if (g_failures == 0) {
    std::cout << "Result: ALL PASSED\n";
  } else {
    std::cout << "Result: " << g_failures << " FAILED\n";
  }
  return g_failures;
}
