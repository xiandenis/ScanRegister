#include "point_cloud_io.hpp"

#include <cmath>
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: framed_load_smoke <cloud.las|.slas|.pcd|.ply>\n";
    return 2;
  }
  const std::string path = argv[1];
  Ddx::point_cloud_io::FramedCloud<pcl::PointXYZ> framed;
  if (Ddx::point_cloud_io::load(path, framed) != 0) {
    std::cerr << "load failed: " << path << "\n";
    return 1;
  }
  double max_abs = 0.0;
  for (auto const& p : framed.cloud.points) {
    max_abs = std::max(max_abs, static_cast<double>(std::abs(p.x)));
    max_abs = std::max(max_abs, static_cast<double>(std::abs(p.y)));
    max_abs = std::max(max_abs, static_cast<double>(std::abs(p.z)));
  }
  std::cout << "path=" << path << "\n"
            << "points=" << framed.cloud.size() << "\n"
            << "origin=(" << framed.origin.x() << ", " << framed.origin.y() << ", "
            << framed.origin.z() << ")\n"
            << "max_abs_local=" << max_abs << "\n";
  return 0;
}
