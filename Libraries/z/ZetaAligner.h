#if defined (_MSC_VER)
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <zeta/zeta_io.h>
#include <zeta/zeta_metrics.h>
#include <zeta/zeta_sampling.h>
#include <zeta/zeta_visualizer.h>
#include <zeta/zeta_analysis.h>
#include <zeta/zeta_ransac.h>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/impl/vtk_lib_io.hpp>

#include <chrono>
#include <string>

#include "zeta-features.hpp"
//#include "zeta-gpuFPFH.cpp"


inline uint64_t timeSinceEpochMillisec() {
	using namespace std::chrono;
	return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

inline void load_head_cloud(std::string head_path, zeta::PointCloudPtr head_cloud, zeta::NormalCloudPtr head_normals,
	bool show_performance = false, int numPointsHead = 2000000, float voxelSizeHead = 0.003) {
	
	//Create alignment input
	auto start= timeSinceEpochMillisec();
	zeta::io::loadMeshRandomSample(head_path, head_cloud, head_normals, numPointsHead, voxelSizeHead);
	auto finish = timeSinceEpochMillisec();

	//Performance Metrics
	if (show_performance) {
		std::cout << "Head Cloud and Head Normal Generation: " << finish - start << std::endl;
	}
	
}

inline void load_scene_cloud(std::string scene_path, zeta::PointCloudPtr scene_cloud, zeta::NormalCloudPtr* scene_normals, 
	bool show_performance = false, float voxelSizeScene = 0.003, float sceneSearchRadiusNormals = 0.012) {
	//Load cloud
	auto start_load_cloud = timeSinceEpochMillisec();
	zeta::io::loadCloudFile(scene_path, scene_cloud);

	// Remove outliers
	auto start_outlier_removal = timeSinceEpochMillisec();
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(scene_cloud);
	sor.setMeanK(10);
	sor.setStddevMulThresh(2.0);
	sor.filter(*scene_cloud);

	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setLeafSize(voxelSizeScene, voxelSizeScene, voxelSizeScene);
	vg.setInputCloud(scene_cloud);
	vg.filter(*scene_cloud);

	//Calculate normals
	auto start_normal_calc = timeSinceEpochMillisec();
	*scene_normals = zeta::analysis::calculateNormals(scene_cloud, scene_cloud, sceneSearchRadiusNormals);
	auto end = timeSinceEpochMillisec();

	//Performance Metrics
	if (show_performance) {
		std::cout << "Scene Cloud Generation: " << start_outlier_removal - start_load_cloud << std::endl;
		std::cout << "Outlier Removal: " << start_normal_calc - start_outlier_removal << std::endl;
		std::cout << "Scene Normal Calculation: " << end - start_normal_calc << std::endl;
	}
}


inline Eigen::Matrix4f align(zeta::PointCloudPtr scene_cloud, zeta::NormalCloudPtr scene_normals,
	zeta::PointCloudPtr head_cloud, zeta::NormalCloudPtr head_normals,
	bool use_ICP = true, bool show_performance = false,
	float fpfhRadius = 0.032, float inlierThresh = 0.012, int ransac_blocks = 10 ) {

	//FPFH
//	int test = calculateFPFHgpu(head_cloud, fpfhRadius);

	auto fpfh_head_start = timeSinceEpochMillisec();
	zeta::FPFHCloudPtr featHead = zeta::analysis::calculateFPFH(head_cloud, head_normals, fpfhRadius);
	auto fpfh_scene_start = timeSinceEpochMillisec();
	zeta::NormalCloudPtr scene_normals_temp = zeta::analysis::calculateNormals(scene_cloud, scene_cloud, 0.012);
	zeta::FPFHCloudPtr featScene = zeta::analysis::calculateFPFH(scene_cloud, scene_normals_temp, fpfhRadius);
	
	//Correspondence
	auto corrs_start = timeSinceEpochMillisec();
	zeta::CorrespondencesPtr corrs = zeta::analysis::getCorrespondences(featScene, featHead);
	
	//RANSAC
	auto ransac_start = timeSinceEpochMillisec();
	int inliers = 0; //store number of inliers
	Eigen::Matrix4f sacTransform = Eigen::Matrix4f::Identity(); 
	sacTransform = RANSACgpu(scene_cloud, head_cloud, corrs, 262144, inlierThresh, ransac_blocks, inliers);
	auto ransac_end = timeSinceEpochMillisec();

	//Performance Metrics
	if (show_performance) {
		std::cout << "Head FPFH: " << fpfh_scene_start - fpfh_head_start << std::endl;
		std::cout << "Scene FPFH: " << corrs_start - fpfh_scene_start << std::endl;
		std::cout << "Feature Correspondences: " << ransac_start - corrs_start << std::endl;
		std::cout << "RANSAC: " << ransac_end - ransac_start << std::endl;
	}

	//ICP
	if (use_ICP) {

		//declare cuda pointer for cloud 

		//pass pointer to texture map 
		auto ICP_start = timeSinceEpochMillisec();
		zeta::PointCloudPtr head_ransac(new zeta::PointCloud);
		zeta::PointCloudPtr alignedFinal(new zeta::PointCloud);
		pcl::transformPointCloud(*head_cloud, *head_ransac, sacTransform);
		Eigen::Matrix4f icpTransform = zeta::analysis::icpAlign(scene_cloud, head_ransac, 50000, .003, 0, 0, alignedFinal);
		sacTransform = icpTransform * sacTransform;
		auto ICP_end = timeSinceEpochMillisec();
		std::cout << "ICP: " << ICP_end - ICP_start << std::endl;
	}
	return sacTransform;
}
