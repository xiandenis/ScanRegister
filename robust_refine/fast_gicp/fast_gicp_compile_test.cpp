#include <pcl/point_types.h>

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/impl/lsq_registration_impl.hpp>
#include <fast_gicp/gicp/impl/fast_gicp_impl.hpp>

int main()
{
  fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setNumThreads(1);
  return 0;
}
