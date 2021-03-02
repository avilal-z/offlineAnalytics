#include "include/helpers.hpp"
float dist(Eigen::Vector4f a, Eigen::Vector4f b) {
	if (a.size() != b.size()) {
		std::cout << "not the same size" << std::endl;
		return -10000;
	}
	else {
		float sqr_dist = 0; 
		for (int i = 0; i < 3; i++) {
			sqr_dist += pow(a[i] - b[i], 2);
		}
		return sqrt(sqr_dist);
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

void generateTRE(std::string root, std::string head_path, std::vector<uint64_t> time_stamps, std::vector<Eigen::Matrix4f> tool_pos, zeta::PointCloudPtr head_cloud,
	zeta::NormalCloudPtr head_normals, Eigen::Matrix4f camerawrtTracker, Eigen::Vector4f tip_offset, Eigen::Vector4f ct_tip_pos) {
	//load head and tools
	load_head_cloud(head_path, head_cloud, head_normals);

	//declare arrayy of clouds to save 
	zeta::PointCloudPtr current_cloud(new zeta::PointCloud);
	zeta::PointCloudPtr alignedFinal(new zeta::PointCloud);

	std::string cloud_file_name = root + "/" + std::to_string(time_stamps[0]) + ".pcd";
	std::cout << cloud_file_name << std::endl;

	Eigen::Matrix4f initial_alignment = full_align(cloud_file_name, head_cloud, head_normals);

	//begin rtrack
	Eigen::Matrix4f previous_transform = initial_alignment.inverse();

	//notes for csv
	std::vector<uint64_t> time_since_last_image = {};
	std::vector<uint64_t> rtrack_time = {};
	std::vector<double> preTRE = {};
	std::vector<double> postTRE = {};
	std::vector<double> tip_movement = {};
	std::vector<Eigen::Matrix4f> transformationMat = {};

	for (int i = 0; i < time_stamps.size(); ++i) {
		cloud_file_name = root + "/" + std::to_string(time_stamps[i]) + ".pcd";
		std::cout << cloud_file_name << std::endl;

		zeta::io::loadCloudFile(cloud_file_name, current_cloud);
		pcl::transformPointCloud(*current_cloud, *current_cloud, previous_transform);

		//calculate pre retrack TRE
		Eigen::Vector4f pre_rtrack_tip_pos = (previous_transform * camerawrtTracker.inverse() * tool_pos[i] * tip_offset);

		//run rtrack
		std::vector<float> distances = {};
		std::vector<int> out_indices = {};
		auto t1 = timeSince();
		Eigen::Matrix4f icp_transform = gpuICPAlign<zeta::Point>(current_cloud, head_cloud, 10000, .003, 0, 0, 1, alignedFinal, distances, out_indices);
		pcl::transformPointCloud(*current_cloud, *current_cloud, icp_transform.inverse());
		auto t2 = timeSince();
		std:cout << "rt time: " << t2 - t1 << std::endl;

		//update data
		pcl::transformPointCloud(*current_cloud, *current_cloud, icp_transform.inverse());
		previous_transform = icp_transform.inverse() * previous_transform;

		std::cout << previous_transform.inverse()(0, 0) << ">" << previous_transform.inverse()(0, 1) << ">" << previous_transform.inverse()(0, 2) << ">" << previous_transform.inverse()(0, 3) << "|"
				  << previous_transform.inverse()(2, 0) << ">" << previous_transform.inverse()(2, 1) << ">" << previous_transform.inverse()(2, 2) << ">" << previous_transform.inverse()(2, 3) << "|"
				  << previous_transform.inverse()(1, 0) << ">" << previous_transform.inverse()(1, 1) << ">" << previous_transform.inverse()(1, 2) << ">" << previous_transform.inverse()(1, 3) << "|"
				  << previous_transform.inverse()(3, 0) << ">" << previous_transform.inverse()(3, 1) << ">" << previous_transform.inverse()(3, 2) << ">" << previous_transform.inverse()(3, 3) << std::endl;

		//log time since last capture
		if (i == 0) {
			time_since_last_image.push_back(0);
		}
		else {
			time_since_last_image.push_back(time_stamps[i] - time_stamps[i - 1]);
		}

		Eigen::Vector4f post_rtrack_tip_pos = (previous_transform * camerawrtTracker.inverse() * tool_pos[i] * tip_offset);

		rtrack_time.push_back(t2 - t1);
		transformationMat.push_back(previous_transform);
		preTRE.push_back(dist(ct_tip_pos, pre_rtrack_tip_pos));
		postTRE.push_back(dist(ct_tip_pos, post_rtrack_tip_pos));
		tip_movement.push_back(dist(tool_pos[0].col(3), tool_pos[i].col(3)));
	}

	//write to csv
	std::cout << "writing results to: " << root << std::endl;
	std::ofstream outfile;
	outfile.open(root + "/rTrackResults.csv");
	outfile << "Pre rtrack TRE" << "," << "Post rtrack TRE" << "," << "Tip distance from starting point" << "," <<  "Time for Rtrack" << "," << "Time Since Last Image" << "," << std::endl;
	for (int i = 0; i < rtrack_time.size(); i++) {
		outfile << std::to_string(preTRE[i]) << ","
			<< std::to_string(postTRE[i]) << ","
			<<	std::to_string(tip_movement[i]) << "," 
			<< std::to_string(rtrack_time[i]) << ","
			<< std::to_string(time_since_last_image[i]) << std::endl;
	}
	outfile.close();

}

void main() {
	//inputs
	std::string root = "D:/CadaverStudyDay2/Head1/head1_roll";
	std::string head_path = "C:/Users/ZImaging/Desktop/data/cadaver post ct models/Head-1-Post.obj";
	Eigen::Matrix4f camera_wrt_tracker = Eigen::Matrix4f::Identity();
	camera_wrt_tracker = matrixify("0.0036723>0.987251>0.159128>0.155323|0.999739>-3.53493e-05>-0.0228523>0.173286|-0.0225553>0.159171>-0.986993>-0.0949453");
	//camera_wrt_tracker << 0.0034366, 0.987197, 0.15947, 0.154069,
	//	0.999733, 0.00024885, -0.023085, 0.173023,
	//	-0.0228291, 0.159506, -0.986933, -0.0945234,
	//	0, 0, 0, 1;

	//Eigen::Vector4f tool_origin_offest = { -0.015289 , 0.000364 , -0.086528, 1 }; // tip test 1
	Eigen::Vector4f tool_origin_offest = { -0.015107 , 0.000331 , -0.08679, 1 }; // tip test 1


	Eigen::Vector4f ct_tip_pos = { 0.006070, -0.186640, 1.262069, 1 }; 


	//load timestamps and poses
	std::vector<uint64_t> time_stamps;
	std::vector<Eigen::Matrix4f> tool_pos;
	csv_reader(root + "/positions.csv", time_stamps, tool_pos);
	auto p = sort_permutation(time_stamps,
		[](uint64_t const& a, uint64_t const& b) {return a < b; });
	time_stamps = apply_permutation(time_stamps, p);
	tool_pos = apply_permutation(tool_pos, p);

	//Generate TRE
	zeta::PointCloudPtr head_cloud(new zeta::PointCloud);
	zeta::NormalCloudPtr head_normals(new zeta::NormalCloud);
	load_head_cloud(head_path, head_cloud, head_normals);
	generateTRE(root, head_path, time_stamps, tool_pos, head_cloud, head_normals, camera_wrt_tracker, tool_origin_offest, ct_tip_pos);
}


