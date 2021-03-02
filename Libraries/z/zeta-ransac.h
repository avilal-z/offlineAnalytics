#pragma once
#include <pcl/gpu/features/features.hpp>
#include <pcl/gpu/containers/kernel_containers.h>
#include <pcl/point_types.h>
#include <thrust/host_vector.h>


void computeRansac(int indices_size, unsigned int sample_size, const unsigned max_skip, pcl::PointXYZ* TargetCloud,
	pcl::PointXYZ* SourceCloud,int* TargetIndices, int* SourceIndices, float inlier_threshold, unsigned int model_size,
	int* Inliers, Eigen::Matrix<float, 16, 1>* Coeffs, int* shuffledIndices, float considerasZero, float precision);

//void maxInlier(int* array, float* max, int* mutex, unsigned int n);