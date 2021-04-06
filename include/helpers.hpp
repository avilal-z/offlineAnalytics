#define FLANN_USE_CUDA 
#include <zeta/zeta_io.h>
#include <zeta/zeta_types.h>
#include <Libraries/z/ZetaAligner.h>
#include "Libraries/Phoxi/phoxi_grabber.h"

#include <flann/flann.h>
#include "Libraries/z/gpuKNN/zeta_cuda_index.h"
#include "Libraries/z/gpuKNN/zeta-gpuICP.hpp"

Eigen::Vector3f rottoeuler(Eigen::Matrix4f R)
{
	float sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));

	bool singular = sy < 1e-6; // If

	float x, y, z;
	if (!singular)
	{
		x = atan2(R(2, 1), R(2, 2));
		y = atan2(-R(2, 0), sy);
		z = atan2(R(1, 0), R(0, 0));
	}
	else
	{
		x = atan2(-R(1, 2), R(1, 1));
		y = atan2(-R(2, 0), sy);
		z = 0;
	}
	return 180 * Eigen::Vector3f(x, y, z) / 3.14159265;
}

std::string stringify(Eigen::Matrix4f a) {
	std::string mat_str = "";
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
		
			mat_str += std::to_string(a(i, j)) + ">";
		
		}
		mat_str += "|";
	}
	return mat_str;
}

Eigen::Matrix4f full_align_txt(zeta::PointCloudPtr segmented_cloud, zeta::PointCloudPtr head_cloud, zeta::NormalCloudPtr head_normals) {
	//parameters
	double voxelSizeScene = 0.003;
	double sceneSearchRadiusNormals = 0.012;
	bool use_ICP = false;
	bool show_performance = true;
	double fpfhRadius = 0.032;
	double inlierThresh = 0.012;
	int ransac_blocks = 10;

	//run full alignment on starting cloud
	zeta::PointCloudPtr alignment_segment_cloud(new zeta::PointCloud);
	zeta::NormalCloudPtr alignment_segment_normals(new zeta::NormalCloud);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(segmented_cloud);
	sor.setMeanK(10);
	sor.setStddevMulThresh(2.0);
	sor.filter(*alignment_segment_cloud);

	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setLeafSize(voxelSizeScene, voxelSizeScene, voxelSizeScene);
	vg.setInputCloud(alignment_segment_cloud);
	vg.filter(*alignment_segment_cloud);

	//Calculate normals
	alignment_segment_normals = zeta::analysis::calculateNormals(alignment_segment_cloud, alignment_segment_cloud, sceneSearchRadiusNormals);
	Eigen::Matrix4f transformationMatrix = align(alignment_segment_cloud, alignment_segment_normals, head_cloud, head_normals, use_ICP, show_performance,
		fpfhRadius, inlierThresh, ransac_blocks);
	return transformationMatrix;
}

Eigen::Matrix4f full_align(std::string cloud_file, zeta::PointCloudPtr head_cloud, zeta::NormalCloudPtr head_normals) {
	//parameters
	double voxelSizeScene = 0.003;
	double sceneSearchRadiusNormals = 0.012;
	bool use_ICP = false;
	bool show_performance = true;
	double fpfhRadius = 0.032;
	double inlierThresh = 0.012;
	int ransac_blocks = 10;

	//run full alignment on starting cloud
	zeta::PointCloudPtr segmented_cloud(new zeta::PointCloud);
	zeta::PointCloudPtr alignment_segment_cloud(new zeta::PointCloud);
	zeta::NormalCloudPtr alignment_segment_normals(new zeta::NormalCloud);

	zeta::io::loadCloudFile(cloud_file, segmented_cloud);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(segmented_cloud);
	sor.setMeanK(10);
	sor.setStddevMulThresh(2.0);
	sor.filter(*alignment_segment_cloud);

	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setLeafSize(voxelSizeScene, voxelSizeScene, voxelSizeScene);
	vg.setInputCloud(alignment_segment_cloud);
	vg.filter(*alignment_segment_cloud);

	//Calculate normals
	alignment_segment_normals = zeta::analysis::calculateNormals(alignment_segment_cloud, alignment_segment_cloud, sceneSearchRadiusNormals);
	Eigen::Matrix4f transformationMatrix = align(alignment_segment_cloud, alignment_segment_normals, head_cloud, head_normals, use_ICP, show_performance,
		fpfhRadius, inlierThresh, ransac_blocks);
	return transformationMatrix;
}

Eigen::Matrix4f matrixify(std::string in) {

	in = in + ",";
	Eigen::Matrix4f m;

	std::replace(in.begin(), in.end(), '>', ',');
	std::replace(in.begin(), in.end(), '|', ',');


	size_t pos = 0;
	std::string token;
	std::string delimiter = ",";
	std::vector<float> holding;

	while ((pos = in.find(delimiter)) != std::string::npos) {
		token = in.substr(0, pos);
		holding.push_back(stof(token));
		in.erase(0, pos + delimiter.length());
	}

	m << holding[0], holding[1], holding[2], holding[3],
		holding[4], holding[5], holding[6], holding[7],
		holding[8], holding[9], holding[10], holding[11],
		0, 0, 0, 1;

	return m;
}

void csv_reader(std::string file_path, std::vector<uint64_t>&time_stamp, std::vector<Eigen::Matrix4f> &tool_pos) {
	//add the rest of the points 
	std::ifstream str(file_path);
	std::string                line;
	int rows = -1;

	while (std::getline(str, line)) {
		std::vector<std::string>   result;
		std::stringstream          lineStream(line);
		std::string                cell;
		int linecount = 0;
		int col_num = 0;
		//skip first  rows 
		if (rows != -1) {
			while (std::getline(lineStream, cell, ','))
			{
				//iterate over columns
				result.push_back(cell);
				//save timestamp
				if (col_num == 0) {
					time_stamp.push_back(std::stoull(result[col_num]));
				}
				//save position
				else if (col_num == 1) {
					Eigen::Matrix4f a = matrixify(result[col_num]);
					tool_pos.push_back(a);
				}
				col_num++;
			}

		}

		rows++;
	}

}



void alignment_reader(std::string file_path, std::vector<uint64_t>& time_stamp, std::vector<Eigen::Matrix4f>& alignments, int alignment_col) {
	//add the rest of the points 
	std::ifstream str(file_path);
	std::string                line;
	int rows = -1;

	while (std::getline(str, line)) {
		std::vector<std::string>   result;
		std::stringstream          lineStream(line);
		std::string                cell;
		int linecount = 0;
		int col_num = 0;
		//skip first  rows 
		if (rows != -1) {
			while (std::getline(lineStream, cell, ','))
			{
				//iterate over columns
				result.push_back(cell);
				//save timestamp
				if (col_num == 0) {
					time_stamp.push_back(std::stoull(result[col_num]));
				}
				//save position
				else if (col_num == alignment_col) {
					Eigen::Matrix4f a = matrixify(result[col_num]);
					alignments.push_back(a);
				}
				col_num++;
			}

		}

		rows++;
	}

}




template <typename T, typename Compare>
std::vector<std::size_t> sort_permutation(
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
std::vector<T> apply_permutation(
	const std::vector<T>& vec,
	const std::vector<std::size_t>& p)
{
	std::vector<T> sorted_vec(vec.size());
	std::transform(p.begin(), p.end(), sorted_vec.begin(),
		[&](std::size_t i) { return vec[i]; });
	return sorted_vec;
}
