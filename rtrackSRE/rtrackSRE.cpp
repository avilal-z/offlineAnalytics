#include "include/helpers.hpp"

void txt_reader(std::string file_path, zeta::PointCloudPtr segmented_cloud) {
	//read csv add in points to cloud
	//skip first 2 rows 
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
		pcl::PointXYZ point;
		//std::cout << rows << std::endl;

			while (std::getline(lineStream, cell, ' '))
			{
				//std::cout << col_num << std::endl;
				result.push_back(cell);
				//save timestamp
				if (col_num == 0) {
					//std::cout << result[col_num] << std::endl;
					point.x = std::stof(result[col_num]);
				}
				else if (col_num == 1) {
					point.y = std::stof(result[col_num]);
				}
				//save 3d model transform
				else if (col_num == 2) {
					point.z = std::stof(result[col_num]);
				}
				col_num++;
			}
			segmented_cloud->points.push_back(point);
		

		rows++;
	}

}

void transformClouds(std::string root, std::string head_path, std::vector<uint64_t> time_stamps, zeta::PointCloudPtr head_cloud, zeta::NormalCloudPtr head_normals) {
	//declare arrayy of clouds to save 
	zeta::PointCloudPtr current_cloud(new zeta::PointCloud);
	zeta::PointCloudPtr alignedFinal(new zeta::PointCloud);

	std::string cloud_file_name = root + "/" + std::to_string(time_stamps[0]) + ".pcd";
	load_head_cloud(head_path, head_cloud, head_normals);
	zeta::io::loadCloudFile(cloud_file_name, current_cloud);

	Eigen::Matrix4f initial_alignment = full_align(cloud_file_name, head_cloud, head_normals);

	//begin rtrack
	Eigen::Matrix4f previous_transform = initial_alignment.inverse();

	for (int i = 0; i < time_stamps.size(); ++i) {
		cloud_file_name = root + "/" + std::to_string(time_stamps[i]) + ".pcd";
		zeta::io::loadCloudFile(cloud_file_name, current_cloud);
		pcl::transformPointCloud(*current_cloud, *current_cloud, previous_transform);
		std::vector<float> distances = {};
		std::vector<int> out_indices = {};

		//run rtrack
		auto t1 = timeSince();
		Eigen::Matrix4f icp_transform = gpuICPAlign<zeta::Point>(current_cloud, head_cloud, 100, .003, 0, 0, 1, alignedFinal, distances, out_indices);
		auto t2 = timeSince();
		std:cout << "rt time: " << t2 - t1 << std::endl;


		//transform cloud and save result
		zeta::PointCloudPtr transformed_cloud(new zeta::PointCloud);
		transformed_cloud->width = current_cloud->points.size();
		transformed_cloud->height = 1;
		transformed_cloud->is_dense = false;
		transformed_cloud->points.resize(transformed_cloud->width * transformed_cloud->height);

		pcl::transformPointCloud(*current_cloud, *transformed_cloud, icp_transform.inverse());
		pcl::io::savePCDFileASCII(root + "/transformed/" + std::to_string(time_stamps[i]) + ".pcd", *transformed_cloud);

		previous_transform = icp_transform.inverse() * previous_transform;
	}

}

void writeSRE(std::string filename, std::vector<float> distances, std::vector<int> out_indices, zeta::PointCloudPtr current_cloud ) {

	//remove outliers from distance cloud and calculate mean 
	std::vector<float> pruned_dist = {};
	std::vector<int> pruned_indices = {};
	float mean_dist = 0;
	for (int i = 0; i < distances.size(); i++) {
		float dist = abs(distances[i]);
		if (isnan(dist) || dist >= 2) { continue; }
		else {
			mean_dist += dist;
			pruned_dist.push_back(dist);
			pruned_indices.push_back(out_indices[i]);
		}
	}
	mean_dist = mean_dist / pruned_dist.size();

	//calculate std dev 
	double std_dev = 0; 
	for (int i = 0; i < pruned_dist.size(); i++) {
		std_dev += pow(pruned_dist[i] - mean_dist, 2);
	}
	std_dev = sqrt(std_dev / pruned_dist.size());

	std::cout << "mean " << mean_dist << std::endl;
	std::cout << "std " << std_dev << std::endl;


	//remove outliers(greater than 3 std dev from the mean)
	//write SRE
	std::ofstream outfile;
	outfile.open(filename);
	for (int i = 0; i < pruned_dist.size(); i++) {
		if(pruned_dist[i] > mean_dist + 2 *std_dev || pruned_dist[i] < mean_dist - 2 * std_dev){
			std::cout << "dist" << pruned_dist[i] << std::endl;
			continue;
		}
		else {
			outfile << current_cloud->points[pruned_indices[i]].x << "," <<
					   current_cloud->points[pruned_indices[i]].y << "," <<
					   current_cloud->points[pruned_indices[i]].z << "," <<
					   distances[i] << std::endl;
		}
	}
	outfile.close();
}

void generateSRE(std::string root, std::string head_path, std::vector<uint64_t> time_stamps, zeta::PointCloudPtr head_cloud, zeta::NormalCloudPtr head_normals) {

	load_head_cloud(head_path, head_cloud, head_normals);

	//declare arrayy of clouds to save 
	zeta::PointCloudPtr current_cloud(new zeta::PointCloud);
	zeta::PointCloudPtr alignedFinal(new zeta::PointCloud);

	std::string cloud_file_name = root + "/" + std::to_string(time_stamps[0]) + ".txt";
	std::cout << cloud_file_name << std::endl;
	zeta::io::loadCloudFile(cloud_file_name, current_cloud);

	Eigen::Matrix4f initial_alignment = full_align_txt(current_cloud, head_cloud, head_normals);

	//begin rtrack
	Eigen::Matrix4f previous_transform = initial_alignment.inverse();

	for (int i = 0; i < time_stamps.size(); ++i) {
		cloud_file_name = root + "/" + std::to_string(time_stamps[i]) + ".txt";
		std::cout << cloud_file_name << std::endl;

		zeta::io::loadCloudFile(cloud_file_name, current_cloud);
		pcl::transformPointCloud(*current_cloud, *current_cloud, previous_transform);
		std::vector<float> distances = {};
		std::vector<int> out_indices = {};

		//run rtrack
		auto t1 = timeSince();
		Eigen::Matrix4f icp_transform = gpuICPAlign<zeta::Point>(current_cloud, head_cloud, 100, .003, 0, 0, 1, alignedFinal, distances, out_indices);
		auto t2 = timeSince();
		std:cout << "rt time: " << t2 - t1 << std::endl;
		pcl::transformPointCloud(*current_cloud, *current_cloud, icp_transform.inverse());

		//save cloud with distances 
		previous_transform = icp_transform.inverse() * previous_transform;
		std::ofstream outfile;
		outfile.open(root + "/alignment.csv");
		outfile << previous_transform(0, 0) << ">" << previous_transform(0, 1) << ">" << previous_transform(0, 2) << ">" << previous_transform(0, 3) << "|"
				<< previous_transform(1, 0) << ">" << previous_transform(1, 1) << ">" << previous_transform(1, 2) << ">" << previous_transform(1, 3) << "|"
				<< previous_transform(2, 0) << ">" << previous_transform(2, 1) << ">" << previous_transform(2, 2) << ">" << previous_transform(2, 3) << "|"
				<< previous_transform(3, 0) << ">" << previous_transform(3, 1) << ">" << previous_transform(3, 2) << ">" << previous_transform(3, 3) << ",";
		outfile.close();
		std::cout << "alignment printed to:" << root << std::endl;
		std::string filename = root + "/out/" + std::to_string(time_stamps[i]) + ".csv";

		writeSRE(filename, distances, out_indices, current_cloud);

	}


}

void main() {
	//load data 
	std::vector<uint64_t> time_stamps;
	std::vector<Eigen::Matrix4f> tool_pos;
	std::string root = "D:/CadaverStudyDay2/Head1/head1_pitch/sreTest";
	std::string head_path = "C:/Users/ZImaging/Desktop/data/Head-1/Head-1_AlignmentSurface.obj";

	//Gather timestamps and tool positions
	csv_reader(root + "/positions.csv", time_stamps, tool_pos);

	//sort in chronological order
	auto p = sort_permutation(time_stamps,
		[](uint64_t const& a, uint64_t const& b) {return a < b; });
	time_stamps = apply_permutation(time_stamps, p);
	tool_pos = apply_permutation(tool_pos, p);

	//load head obj
	zeta::PointCloudPtr head_cloud(new zeta::PointCloud);
	zeta::NormalCloudPtr head_normals(new zeta::NormalCloud);

	//transformClouds(root, head_path, time_stamps, head_cloud, head_normals);
	generateSRE(root, head_path, time_stamps, head_cloud, head_normals);
}

