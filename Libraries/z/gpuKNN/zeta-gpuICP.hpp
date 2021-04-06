#pragma once
#include "zeta_cuda_index.h"
#include <pcl/impl/point_types.hpp>

template<typename PointT>
inline void pointcloud_toFlannMat(typename pcl::PointCloud<PointT>::Ptr cloud, flann::Matrix<float> data, int n_points) {

	for (int i = 0; i < n_points; i++)
	{
		data[i][0] = cloud->points[i].x;
		data[i][1] = cloud->points[i].y;
		data[i][2] = cloud->points[i].z;

	}

}

//target is scene
//source is head obj

//ref is scene 
//query is head

template<typename PointT>
Eigen::Matrix4f gpuICPAlign(typename pcl::PointCloud<PointT>::Ptr target, typename pcl::PointCloud<PointT>::Ptr source, int maxIters, double maxCorrDist, double transformEpsilon, double fitnessEpsilon, int max_nn, 
	typename pcl::PointCloud<PointT>::Ptr outputCloud)
{
	Eigen::Matrix4f final_transformation = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f curr_transformation = Eigen::Matrix4f::Identity();
	pcl::CorrespondencesPtr corrs;

	pcl::registration::TransformationEstimationSVD<PointT, PointT, float> transformation_estimation_;

	//define Flann structures
	int ref_n_points = target->points.size();
	int query_n_points = source->points.size();


	flann::Matrix<float> data ;
	data = flann::Matrix<float>(new float[ref_n_points * 3], ref_n_points, 3);
	flann::Matrix<float> query_mat ;
	flann::Matrix<float> dists ;
	flann::Matrix<int> indices ;
	query_mat = flann::Matrix<float>(new float[query_n_points * 3], query_n_points, 3);
	dists = flann::Matrix<float>(new float[query_mat.rows * max_nn], query_mat.rows, max_nn);
	indices = flann::Matrix<int>(new int[query_mat.rows * max_nn], query_mat.rows, max_nn);
	
	//copy input to appropriate data structure
	pointcloud_toFlannMat<PointT>(target, data, ref_n_points);

	//create tree
	KDTreeCuda3dIndex<flann::L2<float> >index(data, flann::KDTreeCuda3dIndexParams(15));
	index.buildIndex();

	//allocate cloud to be transformed 
	outputCloud->resize(source->points.size());
	outputCloud->header = source->header;
	outputCloud->is_dense = source->is_dense;
	for (int i = 0; i < outputCloud->points.size(); ++i)
	{
		outputCloud->points[i] = source->points[i];
		outputCloud->points[i].data[3] = 1.0;
	}

	bool converged = false;
	int numIters = 0;
	double prev_mse = std::numeric_limits<double>::max();
	double curr_mse = std::numeric_limits<double>::max();

	do {
		Eigen::Matrix4f previous_transformation = curr_transformation;

		corrs.reset(new pcl::Correspondences());
		pointcloud_toFlannMat<PointT>(outputCloud, query_mat, outputCloud->points.size());
		index.knnSearch(query_mat, indices, dists, 1, flann::SearchParams(-1));
		for (int i = 0; i < query_n_points; ++i) {
			if(sqrt(dists[i][0]) <= maxCorrDist) {
				pcl::Correspondence cor;
				cor.index_query = i; 
				cor.index_match = indices[i][0];
				cor.distance = dists[i][0];
				corrs->push_back(cor);
			}
			
		}

		if (corrs->size() < 3)
		{
			std::cout << "ICP couldn't find enough correspondences" << std::endl;
			return final_transformation;
		}
		
		transformation_estimation_.estimateRigidTransformation(*outputCloud, *target, *corrs, curr_transformation);
		pcl::transformPointCloud(*outputCloud, *outputCloud, curr_transformation);
		final_transformation = curr_transformation * final_transformation;


		// Check convergence
		++numIters;
		if (numIters >= maxIters)
		{
			//std::cout << "ICP converged due to reaching maxIters" << std::endl;
			converged = true;
			continue;
		}

		double cos_angle = 0.5 * (curr_transformation.coeff(0, 0) + curr_transformation.coeff(1, 1) +
			curr_transformation.coeff(2, 2) - 1);
		double translation_sqr = curr_transformation.coeff(0, 3) * curr_transformation.coeff(0, 3) +
			curr_transformation.coeff(1, 3) * curr_transformation.coeff(1, 3) +
			curr_transformation.coeff(2, 3) * curr_transformation.coeff(2, 3);

		if ((translation_sqr <= transformEpsilon) &&
			(cos_angle >= 1.0 - transformEpsilon))
		{
			//std::cout << "ICP converged due to transform epsilon" << std::endl;
			converged = true;
			continue;
		}

		curr_mse = 0;
		for (int i = 0; i < corrs->size(); ++i)
		{
			curr_mse += corrs->at(i).distance;
			//std::cout << corrs->at(i).distance << std::endl;
		}
		curr_mse /= double(corrs->size());
		
		if (std::abs(curr_mse - prev_mse) < 1e-12)
		{
			//std::cout << "ICP converged due to absolute MSE" << std::endl;
			converged = true;
			continue;
		}

		if (std::abs(curr_mse - prev_mse) / prev_mse < fitnessEpsilon)
		{

			//std::cout << "ICP converged due to relative MSE" << std::endl;
			converged = true;
			continue;
		}

		prev_mse = curr_mse;

	} while (!converged);

	return final_transformation;
}


Eigen::MatrixXf FindBoundingBox(zeta::PointCloudPtr head_cloud) {
	Eigen::MatrixXf box(2, 3);
	box.row(0) << std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max();
	box.row(1) << std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), std::numeric_limits<float>::min();

	for (int i = 0; i < head_cloud->points.size(); i++) {
		float x = 0;
		float y = 0;
		float z = 0;
		x = head_cloud->points[i].x;
		y = head_cloud->points[i].y;
		z = head_cloud->points[i].z;
		if (x < box(0, 0)) box(0, 0) = x;
		if (x > box(1, 0)) box(1, 0) = x;
		if (y < box(0, 1)) box(0, 1) = y;
		if (y > box(1, 1)) box(1, 1) = y;
		if (z < box(0, 2)) box(0, 2) = z;
		if (z > box(1, 2)) box(1, 2) = z;
	}
	Eigen::MatrixXf cube(4, 8);
	cube.row(0) << box(0, 0), box(0, 0), box(0, 0), box(0, 0), box(1, 0), box(1, 0), box(1, 0), box(1, 0);
	cube.row(1) << box(0, 1), box(0, 1), box(1, 1), box(1, 1), box(0, 1), box(0, 1), box(1, 1), box(1, 1);
	cube.row(2) << box(0, 2), box(1, 2), box(0, 2), box(1, 2), box(0, 2), box(1, 2), box(0, 2), box(1, 2);
	cube.row(3) << 1, 1, 1, 1, 1, 1, 1, 1;

	return cube;
}

Eigen::MatrixXf transformBox(Eigen::MatrixXf cube, Eigen::Matrix4f transform) {
	Eigen::MatrixXf transfo_cube(4, 8);
	Eigen::Vector4f min;
	Eigen::Vector4f max;
	int d = 3;
	transfo_cube = transform * cube;

	for (int j = 0; j < d; j++) {
		min[j] = std::numeric_limits<float>::max();
		max[j] = std::numeric_limits<float>::min();
		for (int k = 0; k < transfo_cube.cols(); k++) {
			if (transfo_cube(j, k) < min[j]) {
				min[j] = transfo_cube(j, k);
			}
			if (transfo_cube(j, k) > max[j]) {
				max[j] = transfo_cube(j, k);
			}
		}
	}

	Eigen::MatrixXf newBox(2, 3);
	newBox.row(0) = min;
	newBox.row(1) = max;
	return newBox;
}

inline Eigen::MatrixXf findBounds(Eigen::MatrixXf cube) {
	Eigen::Vector4f min;
	Eigen::Vector4f max;
	int d = 3;

	for (int j = 0; j < d; j++) {
		min[j] = std::numeric_limits<float>::max();
		max[j] = std::numeric_limits<float>::min();
		for (int k = 0; k < cube.cols(); k++) {
			if (cube(j, k) < min[j]) {
				min[j] = cube(j, k);
			}
			if (cube(j, k) > max[j]) {
				max[j] = cube(j, k);
			}
		}
	}

	Eigen::MatrixXf newBox(2, 3);
	newBox.row(0) = min;
	newBox.row(1) = max;
	return newBox;
}

template<typename PointT>
inline void bindCloud(zeta::PointCloudPtr input, typename pcl::PointCloud<PointT>::Ptr output, Eigen::MatrixXf boundingBox) {
	int size = 0;
	std::vector<PointT> boundPoints = {};
	for (int i = 0; i < input->size(); i++) {
		float x = input->points[i].x;
		float y = input->points[i].y;
		float z = input->points[i].z;
		if (x > boundingBox(0, 0) && x < boundingBox(1, 0) && y > boundingBox(0, 1) && y < boundingBox(1, 1) && z > boundingBox(0, 2) && z < boundingBox(1, 2)) {
			PointT p;
			p.x = input->points[i].x;
			p.y = input->points[i].y;
			p.z = input->points[i].z;
			boundPoints.push_back(p);
			size++;
		}
	}
	output->resize(size);
	for (int i = 0; i < size; i++) {
		output->points[i] = boundPoints[i];
	}
	return;
}


template<typename PointT>
void findDist(zeta::PointCloudPtr target, typename pcl::PointCloud<PointT>::Ptr source, int max_nn, double maxCorrDist,
	std::vector<float>& distances, std::vector<int>& out_indices)
{
	pcl::CorrespondencesPtr corrs;

	//define Flann structures
	int ref_n_points = target->points.size();
	int query_n_points = source->points.size();
	distances.resize(query_n_points);
	out_indices.resize(query_n_points);

	//initialize data structures 
	flann::Matrix<float> data;
	data = flann::Matrix<float>(new float[ref_n_points * 3], ref_n_points, 3);
	flann::Matrix<float> query_mat;
	flann::Matrix<float> dists;
	flann::Matrix<int> indices;
	query_mat = flann::Matrix<float>(new float[query_n_points * 3], query_n_points, 3);
	dists = flann::Matrix<float>(new float[query_mat.rows * max_nn], query_mat.rows, max_nn);
	indices = flann::Matrix<int>(new int[query_mat.rows * max_nn], query_mat.rows, max_nn);

	//copy input to appropriate data structure
	pointcloud_toFlannMat<zeta::Point>(target, data, ref_n_points);

	//create tree
	KDTreeCuda3dIndex<flann::L2<float> >index(data, flann::KDTreeCuda3dIndexParams(15));
	index.buildIndex();

	//find nn
	corrs.reset(new pcl::Correspondences());
	pointcloud_toFlannMat<PointT>(source, query_mat, source->points.size());
	index.knnSearch(query_mat, indices, dists, 1, flann::SearchParams(-1));
	for (int i = 0; i < query_n_points; i++) {
		if (sqrt(dists[i][0]) <= maxCorrDist) {
			distances[i] = dists[i][0];
			out_indices[i] = i;
		}

	}

}

void processDist(std::vector<float> distances, std::vector<int> out_indices, std::vector<float> &pruned_dist, std::vector<int> &pruned_indices, float &mean_dist) {
	pruned_dist = {};
	pruned_indices = {};
	mean_dist = 0;
	for (int i = 0; i < distances.size(); i++) {
		float dist = abs(distances[i]);
		if (isnan(dist) || dist >= 2) { continue; }
		else {
			mean_dist += dist;
			//pruned_dist.push_back(abs(sqrt(dist)));
			pruned_dist.push_back((dist));
			pruned_indices.push_back(out_indices[i]);
		}
	}
	mean_dist /= pruned_dist.size();
}

inline float lerp(float color1, float color2, float frac) {
	return color1 * (1 - frac) + color2 * frac;
}
inline float toRGB(float x) {
	float y = 0;
	if (x <= 0.0031308) {
		y = 12.92 * x;
	}
	else {
		y = (1.055 * pow(x, (1 / 2.4))) - 0.055;
	}
	float return_val = (float)((int)(255.9999 * y));
	if (return_val > 255) {
		return 255;
	}
	else {
		return return_val;
	}
}
inline float fromRGB(float x) {
	float y = 0;
	x /= 255.0;
	if (x <= 0.04045) {
		y = x / 12.92;
	}
	else {
		y = pow(((x + 0.055) / 1.055), 2.4);
	}
	return y;
}

inline void colorHeatMap(zeta::PointCloudColorAPtr cloud, std::vector<float> squared_distances, std::vector<int> out_indices) {

	std::vector<float> rgbColor1 = { 217,250,110 };
	std::vector<float> rgbColor2 = { 255,42,45 };
	int num_steps = 5;

	int step_size = squared_distances.size() / num_steps;
	std::vector<float> linColor1 = { 0,0,0 };
	std::vector<float> linColor2 = { 0,0,0 };
	float sumlinColor1 = 0;
	float sumlinColor2 = 0;

	//normalize RBB inputs
	for (int i = 0; i < rgbColor1.size(); i++) {
		linColor1[i] = fromRGB(rgbColor1[i]);
		linColor2[i] = fromRGB(rgbColor2[i]);
		sumlinColor1 += linColor1[i];
		sumlinColor2 += linColor2[i];
	}


	//assign point color value
	float gamma = .43;
	float bright1 = pow(sumlinColor1, gamma);
	float bright2 = pow(sumlinColor2, gamma);

	float prev_pos = 0;
	int count = 0; 

	for (int i = 0; i < squared_distances.size(); i++) {
		float gradient_pos = 0;
		std::vector<float> color = { 0,0,0 };
		
		if (num_steps == 0) {
			std::cout << "divide by zero" << std::endl;
			return;
		}
		else {
			gradient_pos = (floor(i / step_size)) / num_steps;
			if (gradient_pos == prev_pos) {
				count++;
			}
			else {
				std::cout << "count: " << count << std::endl;
				count = 0;
			}
			prev_pos = gradient_pos;
		}

		//find exact color
		float intensity = pow(lerp(bright1, bright2, gradient_pos), (1 / gamma));
		for (int i = 0; i < linColor1.size(); i++) {
			color[i] = lerp(linColor1[i], linColor2[i], gradient_pos);
		}
		float color_sum = color[0] + color[1] + color[2];
		if (color_sum != 0) {
			for (int i = 0; i < color.size(); i++) {
				color[i] = toRGB((color[i] * intensity) / color_sum);
			}
		}
		cloud->points[out_indices[i]].r = color[0];
		cloud->points[out_indices[i]].g = color[1];
		cloud->points[out_indices[i]].b = color[2];
		//std::cout << gradient_pos << ": " << std::to_string(sqrt(squared_distances[i])) << std::endl;
		 
	}
	
	std::cout << squared_distances.size() << std::endl;
}

template <typename T, typename Compare>
std::vector<std::size_t> sort_permutation1(
	const std::vector<T>& vec,
	Compare& compare)
{
	std::vector<std::size_t> p(vec.size());
	std::iota(p.begin(), p.end(), 0);
	std::sort(p.begin(), p.end(),
		[&](std::size_t i, std::size_t j) { return compare(vec[i], vec[j]); });
	return p;
}

template <typename T>
std::vector<T> apply_permutation1(
	const std::vector<T>& vec,
	const std::vector<std::size_t>& p)
{
	std::vector<T> sorted_vec(vec.size());
	std::transform(p.begin(), p.end(), sorted_vec.begin(),
		[&](std::size_t i) { return vec[i]; });
	return sorted_vec;
}

inline std::vector<float> createHeatMap(zeta::PointCloudColorAPtr scene, zeta::PointCloudPtr head, double maxCorrDist) {
	//find point to point squared dist between cloud 1 and cloud 2
	std::vector<float> square_distances = {};
	std::vector<int> out_indices = {};
	findDist<zeta::PointColorA>(head, scene, 1, maxCorrDist, square_distances, out_indices);

	//remove nans from distance vector and calculate square root of distances
	float mean = 0;
	std::vector<float> pruned_dist = {};
	std::vector<int> pruned_indices = {};
	processDist(square_distances, out_indices, pruned_dist, pruned_indices, mean);
	std::cout << "mean" <<  mean << std::endl;

	//sort distances from least to greatest
	auto p = sort_permutation1(pruned_dist,
		[](float const& a, float const& b) {return a < b; });
	pruned_dist = apply_permutation1(pruned_dist, p);
	pruned_indices = apply_permutation1(pruned_indices, p);


	//color cloud based on distances 
	colorHeatMap(scene, pruned_dist, pruned_indices);


	return pruned_dist;
}