#pragma once
#include <Eigen/Core>

namespace teaser
{
	class TeaserAlign
	{
	public:
		typedef Eigen::Matrix<double, 3, Eigen::Dynamic> PointCloud;

		enum class Method
		{
			FGNC_TLS = 0,
			/*	FFGR = 1,*/
			FQUATRO = 2,
		};
	public:


		TeaserAlign();

		void setSourceCloud(PointCloud const& sourceCloud);
		void setTargetCloud(PointCloud const& targeCloud);
		void setDSampleRadius(double dSampleRadius);
		void setMethod(Method type);

		bool align();

		Eigen::Matrix4d getFinalTransformationMatrix();

	private:
		PointCloud m_source_cloud;
		PointCloud m_target_cloud;
		double m_dSampleRadius;
		Eigen::Matrix4d m_transformation_matrix;

		Method m_method;
	};
}

