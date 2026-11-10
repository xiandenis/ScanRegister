#include "teaser_respondence.hpp"
#include "matcher.h"

using namespace teaser;

std::vector<std::pair<int, int>>
TeaserRespondence::calculateCorrespondences(const EPointCloud& source_points, const EPointCloud& target_points,
	FPFHDescriptorPtr& source_features, FPFHDescriptorPtr& target_features)
{

	teaser::PointCloud source_cloud;
	source_cloud.reserve(source_points.cols());
	for (int i = 0; i < source_points.cols(); ++i)
	{
		const Eigen::Vector3d p_i = source_points.col(i);
		PointXYZ p_xyz;
		p_xyz.x = p_i[0];
		p_xyz.y = p_i[1];
		p_xyz.z = p_i[2];
		source_cloud.push_back(p_xyz);
	}


	teaser::PointCloud target_cloud;
	target_cloud.reserve(target_points.cols());
	for (int i = 0; i < target_points.cols(); ++i)
	{
		const Eigen::Vector3d p_i = target_points.col(i);
		PointXYZ p_xyz;
		p_xyz.x = p_i[0];
		p_xyz.y = p_i[1];
		p_xyz.z = p_i[2];
		target_cloud.push_back(p_xyz);
	}

	teaser::Matcher matcher;
	bool use_absolute_scale = true,
		use_crosscheck = true,
		use_tuple_test = true; // false;
	float tuple_scale = 0.85;
	return matcher.calculateCorrespondences(
		source_cloud, target_cloud,
		*source_features, *target_features,
		use_absolute_scale, use_crosscheck, use_tuple_test, tuple_scale);
}

std::vector<std::pair<int, int>>
TeaserRespondence::calculateCorrespondences(const PointCloudPtr& source_points, const PointCloudPtr& target_points,
	FPFHDescriptorPtr& source_features, FPFHDescriptorPtr& target_features)
{
	teaser::PointCloud source_cloud;
	source_cloud.reserve(source_points->size());
	for (int i = 0; i < source_points->size(); ++i)
	{
		PointXYZ p_xyz;
		p_xyz.x = source_points->points[i].x;
		p_xyz.y = source_points->points[i].y;
		p_xyz.z = source_points->points[i].z;
		source_cloud.push_back(p_xyz);
	}


	teaser::PointCloud target_cloud;
	target_cloud.reserve(target_points->size());
	for (int i = 0; i < target_points->size(); ++i)
	{
		PointXYZ p_xyz;
		p_xyz.x = target_points->points[i].x;
		p_xyz.y = target_points->points[i].y;
		p_xyz.z = target_points->points[i].z;
		target_cloud.push_back(p_xyz);
	}

	teaser::Matcher matcher;
	bool use_absolute_scale = true,
		use_crosscheck = true,
		use_tuple_test = true; // false;
	float tuple_scale = 0.85;
	return matcher.calculateCorrespondences(
		source_cloud, target_cloud,
		*source_features, *target_features,
		use_absolute_scale, use_crosscheck, use_tuple_test, tuple_scale);
}
