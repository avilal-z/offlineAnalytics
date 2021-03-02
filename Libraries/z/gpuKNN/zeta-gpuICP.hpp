#pragma once
#include "zeta_cuda_index.h"
#include <pcl/impl/point_types.hpp>

inline void pointcloud_toFlannMat(zeta::PointCloudPtr cloud, flann::Matrix<float> data, int n_points) {

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
	typename pcl::PointCloud<PointT>::Ptr outputCloud, std::vector<float>& distances, std::vector<int> &out_indices)
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
	pointcloud_toFlannMat(target, data, ref_n_points);

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
		pointcloud_toFlannMat(outputCloud, query_mat, outputCloud->points.size());
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


	for (int i = 0; i < query_n_points; i++) {
		distances.push_back(dists[i][0]);
		out_indices.push_back(indices[i][0]);
	}

	return final_transformation;
}