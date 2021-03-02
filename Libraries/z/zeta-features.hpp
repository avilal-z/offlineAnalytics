#include<string>
#include <pcl/gpu/features/features.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/gpu/containers/kernel_containers.h>
#include <pcl/search/search.h>
#include <Eigen/StdVector>
#include <algorithm>
#include <omp.h>
#include <mutex>
#include "zeta-ransac.h"


inline uint64_t timeSince() {
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

struct DataSource
		{
			int max_nn_size;
			float radius;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
			std::vector<pcl::PointXYZ> normals;
			std::vector< std::vector<int> > neighbors_all;
			std::vector<int> sizes;
			
			DataSource() {

			}

			inline void putCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cld) {
				cloud = cld;
			}

			inline void putNormals(std::vector<pcl::PointXYZ> norm) {
				normals = norm;
			}

			inline void findRadiusNeghbors(float radius)
			{
				//radius = radius == -1 ? this->radius : radius;

				pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
				kdtree->setInputCloud(cloud);
				size_t cloud_size = cloud->points.size();
				std::vector<float> dists;
				std::vector<int> current_neighbors;
				neighbors_all.resize(cloud_size);
				sizes.resize((int)cloud_size);
				double radius_temp = radius;

				//loop
				//#pragma omp parallel for private(dists, current_neighbors, radius_temp)
				for (size_t i = 0; i < cloud_size; ++i)
				{			
					kdtree->radiusSearch(cloud->points[i], radius, current_neighbors, dists);
					neighbors_all[i] = current_neighbors;
					sizes[i] = ((int)neighbors_all[i].size());
					radius_temp = radius;
				}
				max_nn_size =* max_element(sizes.begin(), sizes.end());
			}

			inline void getNeghborsArray(std::vector<int>& data)
			{
				data.resize(max_nn_size * neighbors_all.size());
				pcl::gpu::PtrStep<int> ps(&data[0], max_nn_size * sizeof(int));
				//#pragma omp parallel for
				for (size_t i = 0; i < neighbors_all.size(); ++i)
					copy(neighbors_all[i].begin(), neighbors_all[i].end(), ps.ptr(i));
			}
		};

inline std::vector<pcl::PointXYZ> tovector(pcl::PointCloud<pcl::PointXYZ>::Ptr sample1) {
	std::vector<pcl::PointXYZ> returnval;
	returnval.resize(sample1->points.size());
	for (int i = 0; i < sample1->size(); ++i)
	{
		returnval[i].x = sample1->points[i].x;
		returnval[i].y = sample1->points[i].y;
		returnval[i].z = sample1->points[i].z;
	}
	return returnval;
}

inline pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh2pointcloud(DataSource source, std::vector<pcl::FPFHSignature33> input) {

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr result_cloud(new pcl::PointCloud<pcl::FPFHSignature33>());
	result_cloud->points.resize(source.cloud->points.size());
	for (size_t i = 0; i < source.cloud->points.size(); ++i)
	{
		result_cloud->points[i] = input[i];
	}
	result_cloud->width = source.cloud->width;
	result_cloud->height = source.cloud->height;
	result_cloud->is_dense = source.cloud->is_dense;
	return result_cloud;
}

inline std::vector<pcl::PointXYZ> calculateNormalsgpu2(pcl::PointCloud<pcl::PointXYZ>::Ptr shared_cloud, float fpfhRadius) {
	//Normals on GPU
	std::vector<int> data;
	pcl::gpu::NormalEstimation::PointCloud cloud;
	pcl::gpu::NormalEstimation::Normals normals;
	pcl::gpu::NeighborIndices indices1;
	std::vector<pcl::PointXYZ> downloaded1;
	DataSource source;

	//initialize source
	source.putCloud(shared_cloud);
	source.findRadiusNeghbors(fpfhRadius);
	source.getNeghborsArray(data);

	// convert to single array 
	cloud.upload(shared_cloud->points);
	std::vector<int> neighbors_all(source.max_nn_size * cloud.size());
	pcl::gpu::PtrStep<int> ps(&neighbors_all[0], source.max_nn_size * pcl::gpu::PtrStep<int>::elem_size);
	for (size_t i = 0; i < cloud.size(); ++i)
	{
		copy(source.neighbors_all[i].begin(), source.neighbors_all[i].end(), ps.ptr(i));
	}
	indices1.upload(neighbors_all, source.sizes, source.max_nn_size);

	float mark0 = timeSince();
	//gpu computations
	pcl::gpu::NormalEstimation::computeNormals(cloud, indices1, normals);
	pcl::gpu::NormalEstimation::flipNormalTowardsViewpoint(cloud, 0.f, 0.f, 0.f, normals);
	normals.download(downloaded1);
	source.putNormals(downloaded1);
	
	return downloaded1;

}

inline pcl::PointCloud<pcl::FPFHSignature33>::Ptr calculateFPFHgpu2(pcl::PointCloud<pcl::PointXYZ>::Ptr shared_cloud,
														    float fpfhRadius, std::vector<pcl::PointXYZ> normals) {

	int stub;
	pcl::gpu::FPFHEstimation::PointCloud cloud_gpu;
	pcl::gpu::FPFHEstimation::Normals normals_gpu;
	pcl::gpu::NeighborIndices indices;
	pcl::gpu::FPFHEstimation fpfh_gpu;
	pcl::gpu::DeviceArray2D<pcl::FPFHSignature33> fpfh33_features;
	std::vector<pcl::FPFHSignature33> downloaded;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr gpufeatures(new pcl::PointCloud<pcl::FPFHSignature33>());
	DataSource source;
	std::vector<int> data;

	//initialize source
	source.putCloud(shared_cloud);
	source.findRadiusNeghbors(fpfhRadius);
	source.getNeghborsArray(data);
	gpufeatures->points.resize(source.cloud->points.size());

	//GPU processing
	cloud_gpu.upload(source.cloud->points);
	normals_gpu.upload(normals);
	indices.upload(data, source.sizes, source.max_nn_size);
	fpfh_gpu.compute(cloud_gpu, normals_gpu, indices, fpfh33_features);
	fpfh33_features.download(downloaded, stub);

	//transfer vector to point cloud
	for (size_t i = 0; i < source.cloud->points.size(); ++i)
	{
		gpufeatures->points[i] = downloaded[i];
	}
	gpufeatures->width = source.cloud->width;
	gpufeatures->height = source.cloud->height;
	gpufeatures->is_dense = source.cloud->is_dense;

	return gpufeatures;
}

inline Eigen::Matrix4f RANSACgpu(pcl::PointCloud<pcl::PointXYZ>::Ptr target, pcl::PointCloud<pcl::PointXYZ>::Ptr source,
							pcl::CorrespondencesPtr corrs, int maxIters, double inlierThreshold, int iter_power, int &inliers)
{

	int max_iterations_ = std::max(maxIters, 0);
	float threshold_ = (float)inlierThreshold;
	int nr_correspondences = static_cast<int> (corrs->size());
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_ = target;
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_ = source;
	
	int iterations_ = 0;
	int n_best_inliers_count = -INT_MAX;
	std::vector<int> best_model;
	Eigen::Matrix<float, 16, 1> best_model_coefficients;
	
	//Data Structures
	std::vector<int> source_indices(nr_correspondences);
	std::vector<int> target_indices(nr_correspondences);

	//host pointers
	int* TargetIndices = new int[nr_correspondences];
	int *SourceIndices = new int[nr_correspondences];
	int * ShuffledIndices = new int[nr_correspondences]; 
	int *Inliers = new int[maxIters];
	Eigen::Matrix<float, 16, 1>* Coeffs = new Eigen::Matrix<float, 16, 1>[maxIters];
	pcl::PointXYZ* targetCloud = new pcl::PointXYZ[target->size()];
	pcl::PointXYZ* sourceCloud = new pcl::PointXYZ[source->size()]; 

	//device pointers
	int *gpuTargetIndices, *gpuSourceIndices, *gpuShuffledIndices, *gpuInliers;
	Eigen::Matrix<float, 16, 1> *gpuCoeffs;
	pcl::PointXYZ *gpuTargetCloud, *gpuSourceCloud;

	//Pointer Sizes
	int indexPointerSize = nr_correspondences * sizeof(int);
	int inlierPointerSize = maxIters * sizeof(int);
	int coeffPointerSize = maxIters * sizeof(Eigen::Matrix<float, 16, 1>);
	int targetCloudPointerSize = target->size() * sizeof(pcl::PointXYZ);
	int sourceCloudPointerSize = source->size() * sizeof(pcl::PointXYZ);

	//cudaMalloc Device Pointers
	cudaMalloc((void**)&gpuTargetIndices, indexPointerSize);
	cudaMalloc((void**)&gpuSourceIndices, indexPointerSize);
	cudaMalloc((void**)&gpuShuffledIndices, indexPointerSize);
	cudaMalloc((void**)&gpuInliers, inlierPointerSize);
	cudaMalloc((void**)&gpuCoeffs, coeffPointerSize);
	cudaMalloc((void**)&gpuTargetCloud, targetCloudPointerSize);
	cudaMalloc((void**)&gpuSourceCloud, sourceCloudPointerSize);
	
	//populate indices
	for (size_t i = 0; i < corrs->size(); ++i)
	{
		SourceIndices[i] = corrs->at(i).index_query;	
		TargetIndices[i] = corrs->at(i).index_match;

	}

	for (int i = 0; i < target->size(); ++i)
	{
		targetCloud[i].x = target->points[i].x;
		targetCloud[i].y = target->points[i].y;
		targetCloud[i].z = target->points[i].z;	
	}

	for (int i = 0; i < source->size(); ++i)
	{
		sourceCloud[i].x = source->points[i].x;
		sourceCloud[i].y = source->points[i].y;
		sourceCloud[i].z = source->points[i].z;

	}

	//copy from host to device pointers for indices
	cudaMemcpy(gpuTargetIndices, TargetIndices, indexPointerSize, cudaMemcpyHostToDevice);
	cudaMemcpy(gpuSourceIndices, SourceIndices, indexPointerSize, cudaMemcpyHostToDevice);
	cudaMemcpy(gpuShuffledIndices, SourceIndices, indexPointerSize, cudaMemcpyHostToDevice);
	cudaMemcpy(gpuTargetCloud, targetCloud, targetCloudPointerSize, cudaMemcpyHostToDevice);
	cudaMemcpy(gpuSourceCloud, sourceCloud, sourceCloudPointerSize, cudaMemcpyHostToDevice);


	//declare and redeclare model for multithread
	pcl::SampleConsensusModelRegistration<pcl::PointXYZ>::Ptr model;
	model.reset(new pcl::SampleConsensusModelRegistration<pcl::PointXYZ>(input_, source_indices));
	model->setInputTarget(target_, target_indices);
	pcl::SampleConsensusModelRegistration<pcl::PointXYZ> model_inner(*model);

	//More gpu variables
	int indices_size = model->getIndices()->size();
	int sample_size = model->getSampleSize();
	int model_size = model->getModelSize();
	float considerasZero = (std::numeric_limits<float>::min)();
	float precision = 2.0 * Eigen::numext::numeric_limits<float>::epsilon();
	Eigen::Matrix4f best_transformation_;
	int max_inliers = 0;
	int max_inlier_index = 0;
	
	
	//int iter_power;
	int block_size = 512;
	int grid_size = block_size * block_size;
	/*
	if (maxIters > grid_size) {
		maxIters / grid_size;
	}*/



	for (int j = 0; j < iter_power; j++) {
		uint64_t mark0 = timeSince();
		//compute ransac on gpu
		computeRansac(indices_size, sample_size, maxIters,
			gpuTargetCloud, gpuSourceCloud, gpuTargetIndices, gpuSourceIndices,
			inlierThreshold, model_size, gpuInliers, gpuCoeffs, gpuShuffledIndices,
			considerasZero, precision);

		uint64_t mark1 = timeSince();
		//std::cout << "kernel call time: " << mark1 - mark0 << std::endl;

		//copy from device pointer to host
		cudaMemcpy(Coeffs, gpuCoeffs, coeffPointerSize, cudaMemcpyDeviceToHost);
		cudaMemcpy(Inliers, gpuInliers, inlierPointerSize, cudaMemcpyDeviceToHost);
		uint64_t mark25 = timeSince();
		//std::cout << "Copy from device to host: " << mark25 - mark1 << std::endl;


		//find best transformation
		for (int i = 0; i < maxIters; i++) {
			
			//std::cout << "number of inliers :" << Inliers[i] << std::endl;
			if (Inliers[i] > max_inliers) {
				max_inliers = Inliers[i];
				max_inlier_index = i;
				best_transformation_.row(0) = Coeffs[max_inlier_index].segment<4>(0).matrix();
				best_transformation_.row(1) = Coeffs[max_inlier_index].segment<4>(4).matrix();
				best_transformation_.row(2) = Coeffs[max_inlier_index].segment<4>(8).matrix();
				best_transformation_.row(3) = Coeffs[max_inlier_index].segment<4>(12).matrix();
			}
		}

		Eigen::Matrix4f tempMat;
		tempMat.row(0) = Coeffs[0].segment<4>(0).matrix();
		tempMat.row(1) = Coeffs[0].segment<4>(4).matrix();
		tempMat.row(2) = Coeffs[0].segment<4>(8).matrix();
		tempMat.row(3) = Coeffs[0].segment<4>(12).matrix();
	}
	inliers = max_inliers;
	std::cout << "max inliers: " << max_inliers << std::endl;

	//free host pointers
	delete[] TargetIndices;
	delete[] SourceIndices;
	delete[] ShuffledIndices;
	delete[] targetCloud;
	delete[] sourceCloud;

	//free cuda pointers
	cudaFree(gpuTargetIndices);
	cudaFree(gpuSourceIndices);
	cudaFree(gpuShuffledIndices);
	cudaFree(gpuInliers);
	cudaFree(gpuCoeffs);
	cudaFree(gpuSourceCloud);
	cudaFree(gpuTargetCloud);

	delete [] Inliers;
	delete [] Coeffs;

	return best_transformation_;
}