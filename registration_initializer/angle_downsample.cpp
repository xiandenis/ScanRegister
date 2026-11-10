#include "angle_downsample.hpp"

#include <common/panorama_util.hpp>

#include <pcl/common/io.h>

#include <cmath>
#include <limits>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>

namespace Ddx
{

using pcl_type::PointCloud;
using pcl_type::PointCloudPtr;

namespace
{

struct LocalDepthIndexMap
{
	std::vector<float> depth;
	std::vector<int> idx;

	void reset(std::size_t cells)
	{
		depth.assign(cells, std::numeric_limits<float>::max());
		idx.assign(cells, -1);
	}
};

bool collect_winner_indices(PointCloudPtr const& cloud, int width, int height,
                            std::vector<int>& indices)
{
	std::size_t const cells = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
	int const n = static_cast<int>(cloud->size());
	int const grain = std::max(1, n / (8 * std::max(1, tbb::this_task_arena::max_concurrency())));

	tbb::enumerable_thread_specific<LocalDepthIndexMap> tls;
	tbb::parallel_for(tbb::blocked_range<int>(0, n, grain), [&](tbb::blocked_range<int> const& range) {
		LocalDepthIndexMap& local = tls.local();
		if (local.depth.size() != cells)
		{
			local.reset(cells);
		}
		for (int i = range.begin(); i < range.end(); ++i)
		{
			Eigen::Vector3f const p = cloud->points[i].getVector3fMap();
			float const depth = p.norm();
			if (!(depth > 0.f) || !std::isfinite(depth))
			{
				continue;
			}
			Eigen::Vector2i const pixel = PanoramaUtil::math_pixel_from_xyz(p, width, height);
			if (pixel[0] < 0 || pixel[0] >= width || pixel[1] < 0 || pixel[1] >= height)
			{
				continue;
			}
			std::size_t const cell = static_cast<std::size_t>(pixel[1] * width + pixel[0]);
			if (depth < local.depth[cell])
			{
				local.depth[cell] = depth;
				local.idx[cell] = i;
			}
		}
	});

	std::vector<float> merged_depth(cells, std::numeric_limits<float>::max());
	std::vector<int> merged_idx(cells, -1);
	for (auto it = tls.begin(); it != tls.end(); ++it)
	{
		for (std::size_t c = 0; c < cells; ++c)
		{
			if (it->depth[c] < merged_depth[c])
			{
				merged_depth[c] = it->depth[c];
				merged_idx[c] = it->idx[c];
			}
		}
	}

	indices.clear();
	indices.reserve(cells);
	for (std::size_t c = 0; c < cells; ++c)
	{
		if (merged_idx[c] >= 0)
		{
			indices.push_back(merged_idx[c]);
		}
	}
	return !indices.empty();
}

} // namespace

bool AngleDownsample::downsampling(PointCloudPtr const& cloud, std::vector<int>& indices, int width,
                                   int height)
{
	if (cloud == nullptr || cloud->size() < 10 || width <= 0 || height <= 0)
	{
		return false;
	}
	return collect_winner_indices(cloud, width, height, indices);
}

bool AngleDownsample::downsampling(PointCloudPtr const& cloud, PointCloudPtr& ds_cloud, int width,
                                   int height)
{
	std::vector<int> indices;
	if (downsampling(cloud, indices, width, height) == false)
	{
		return false;
	}

	if (indices.empty())
		return false;

	if (ds_cloud == nullptr)
	{
		ds_cloud.reset(new PointCloud());
	}
	pcl::copyPointCloud(*cloud, indices, *ds_cloud);
	return true;
}

} // namespace Ddx
