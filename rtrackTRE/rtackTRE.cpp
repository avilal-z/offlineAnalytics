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

		//run rtrack
		auto t1 = timeSince();
		Eigen::Matrix4f icp_transform = gpuICPAlign<zeta::Point>(current_cloud, head_cloud, 100, .003, 0, 0, 1, alignedFinal);
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
	zeta::NormalCloudPtr head_normals, Eigen::Matrix4f camerawrtTracker, std::vector<Eigen::Vector4f> tip_offset, std::vector<Eigen::Vector4f> ct_tip_pos, int interval, float icp_param, std::string folder) {
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
	std::vector<uint64_t> time_since_directly_last_image = {};
	std::vector<uint64_t> time_since_first_image = {};
	std::vector<uint64_t> rtrack_time = {};
	std::vector<double> preTRE = {};
	std::vector<double> postTRE = {};
	std::vector<double> from_start_tip_movement = {};
	std::vector<double> from_prev_tip_movement = {};
	std::vector<Eigen::Matrix4f> transformationMat = {};
	std::vector<std::string> alignments = {};
	std::vector<int> cum_sum = {};
	int iters = 0;

	for (int i = 0; i < time_stamps.size(); ++i) {
		if (i % interval == 0) {
			cloud_file_name = root + "/" + std::to_string(time_stamps[i]) + ".pcd";
			//std::cout << cloud_file_name << std::endl;

			zeta::io::loadCloudFile(cloud_file_name, current_cloud);
			pcl::transformPointCloud(*current_cloud, *current_cloud, previous_transform);

			//calculate pre retrack TRE
			std::vector<Eigen::Vector4f> pre_rtrack_tip_pos;
			pre_rtrack_tip_pos.resize(tip_offset.size());
			for (int j = 0; j < tip_offset.size(); j++) {
				pre_rtrack_tip_pos[j] = (previous_transform * camerawrtTracker.inverse() * tool_pos[i] * tip_offset[j]);
			}
			

			auto t1 = timeSince();
			Eigen::Matrix4f icp_transform = gpuICPAlign<zeta::Point>(current_cloud, head_cloud, 10000, .003, icp_param, icp_param, 1, alignedFinal);
			pcl::transformPointCloud(*current_cloud, *current_cloud, icp_transform.inverse());
			previous_transform = icp_transform.inverse() * previous_transform;
			auto t2 = timeSince();
			//std:cout << "rt time: " << t2 - t1 << std::endl;

			//update data
			std::string alignment_str = std::to_string(previous_transform.inverse()(0, 0)) + ">" + std::to_string(previous_transform.inverse()(0, 1)) + ">" +std::to_string(previous_transform.inverse()(0, 2)) + ">" + std::to_string(previous_transform.inverse()(0, 3)) + "|" +
										std::to_string(previous_transform.inverse()(1, 0)) + ">" + std::to_string(previous_transform.inverse()(1, 1)) + ">" +std::to_string(previous_transform.inverse()(1, 2)) + ">" + std::to_string(previous_transform.inverse()(1, 3)) + "|" +
										std::to_string(previous_transform.inverse()(2, 0)) + ">" + std::to_string(previous_transform.inverse()(2, 1)) + ">" +std::to_string(previous_transform.inverse()(2, 2)) + ">" + std::to_string(previous_transform.inverse()(2, 3)) + "|" +
										std::to_string(previous_transform.inverse()(3, 0)) + ">" + std::to_string(previous_transform.inverse()(3, 1)) + ">" +std::to_string(previous_transform.inverse()(3, 2)) + ">" + std::to_string(previous_transform.inverse()(3, 3));
			//std::cout << std::to_string(time_stamps[i]) << std::endl;
			
			//std::cout << alignment_str << std::endl;
			
			
			//log time since last capture
			if (i == 0) {
				time_since_last_image.push_back(0);
				time_since_first_image.push_back(0);
				from_prev_tip_movement.push_back(0);
				time_since_directly_last_image.push_back(0);
				cum_sum.push_back(0);
			}
			else {
				time_since_last_image.push_back(time_stamps[i] - time_stamps[(iters-1) * interval]);
				time_since_directly_last_image.push_back(time_stamps[i] - time_stamps[i-1]);
				time_since_first_image.push_back(time_stamps[i] - time_stamps[0]);
				from_prev_tip_movement.push_back(dist(tool_pos[i].col(3), tool_pos[(iters-1) * interval].col(3)));
				cum_sum.push_back(cum_sum[iters-1] + time_since_directly_last_image[iters]);
			}

			std::vector<Eigen::Vector4f> post_rtrack_tip_pos;
			post_rtrack_tip_pos.resize(tip_offset.size());
			for (int j = 0; j < tip_offset.size(); j++) {
				post_rtrack_tip_pos[j] = (previous_transform * camerawrtTracker.inverse() * tool_pos[i] * tip_offset[j]);
				for (int k = 0; k < ct_tip_pos.size(); k++) {
					double pre = dist(ct_tip_pos[k], pre_rtrack_tip_pos[j]);
					double post = dist(ct_tip_pos[k], post_rtrack_tip_pos[j]);
					preTRE.push_back(pre);
					postTRE.push_back(post);
					//std::cout <<  post << std::endl;
					
				}
			}
			rtrack_time.push_back(t2 - t1);
			transformationMat.push_back(previous_transform);
			//std::cout << std::to_string(dist(ct_tip_pos, post_rtrack_tip_pos)) << std::endl;
			from_start_tip_movement.push_back(dist(tool_pos[0].col(3), tool_pos[i].col(3)));
			alignments.push_back(alignment_str);
			iters++;
		}
		else {
			continue;
		}
		
		
	}

	//write to csv
	std::cout << "writing results to: " << root + "/" + folder << std::endl;
	std::string directory_name = root + "/mod" + std::to_string(interval);
	boost::filesystem::path dir(directory_name);
	if (!(boost::filesystem::exists(dir))) {
		std::cout << "Doesn't Exist" << std::endl;
		if (boost::filesystem::create_directory(dir))
			std::cout << "....Successfully Created !" << std::endl;
	}
	std::ofstream outfile;
	std::string titles = "";
	titles += "Timestamp,";		

	for (int j = 0; j < tip_offset.size(); j++) {
		for (int k = 0; k < ct_tip_pos.size(); k++) {
			titles += "Pre rtrack TRE" + std::to_string(j) + std::to_string(k) +
				"," + "Post rtrack TRE" + std::to_string(j) + std::to_string(k) + ",";
		}
	}
	
	titles += 
		"NDI distance from starting point,NDI distance from previous point,Time for Rtrack,Time Since Beginning,Cumulative Time,Time Since Previous Capture,Velocity(cm/s),Alignment Matrix";
	outfile.open(root + "/mod" + std::to_string(interval) + "/rTrackResultsmod" + folder + ".csv");
	outfile << titles << std::endl;
	
	for (int i = 0; i < rtrack_time.size(); i++) {
		std::string out_str = "";
		out_str
			+= std::to_string(time_stamps[i]) + ",";

		int jmax = tip_offset.size();
		int kmax = ct_tip_pos.size();

		for (int j = 0; j < tip_offset.size(); j++) {
			for (int k = 0; k < ct_tip_pos.size(); k++) {
				int index = 0;
				index = (i * (jmax * kmax)) + (j * kmax + k);
				out_str += std::to_string(preTRE[index]) + "," + std::to_string(postTRE[index]) + ",";
				//std::cout << index << std::endl;
			}

		}
		out_str += std::to_string(from_start_tip_movement[i]) + ","
			+ std::to_string(from_prev_tip_movement[i]) + ","
			+ std::to_string(rtrack_time[i]) + ","
			+ std::to_string(time_since_first_image[i]) + ","
			//+ std::to_string(time_since_last_image[i]) + ","
			+ std::to_string(cum_sum[i]) + ","
			+ std::to_string(time_since_directly_last_image[i]) + ","
			+ std::to_string(from_prev_tip_movement[i] * 100 / time_since_directly_last_image[i] * 1000) + ","
			+ alignments[i];
		outfile << out_str << std::endl;
	}
	outfile.close();

}

void main() {
	//inputs
	std::vector<std::string> root;
	std::string head_path;
	Eigen::Matrix4f camera_wrt_tracker = Eigen::Matrix4f::Identity();
	std::vector<Eigen::Vector4f> tool_origin_offest;
	std::vector<Eigen::Vector4f> ct_tip_pos;
	//root.resize(3);

	////03.03 - Phantom Study;
	////std::vector<int> interval = { 1, 50 };
	////std::vector<float> icp_param = { 0, 1e-5, 1e-7, 1e-9, 1e-10, 1e-11, 1e-13 };
	////std::vector<std::string> folder = { "0","1e5","1e7","1e9","1e10","1e11","1e13" };
	//std::vector<int> interval = { 5, 10, 25, 50 };
	//std::vector<float> icp_param = { 1e-7 };
	//std::vector<std::string> params = { "1e7"};
	//std::string angle = "roll";
	//root.resize(1);
	//root[0] = "D:/PhantomStudy/rTrack/" + angle;//no avg
	//head_path = "C:/Users/ZImaging/Desktop/data/Phantom/Phantom_AlignmentSurface.obj";
	//tool_origin_offest.resize(1);
	//tool_origin_offest[0] = { -0.015048, 0.001114, -0.110741, 1 }; //lowered camera 03.03
	////tool_origin_offest =  {-0.016155, 0.001061, -0.110658, 1}; //elevated
	//ct_tip_pos.resize(1);
	//ct_tip_pos[0] = { -0.011979, -0.197292, -0.122339, 1 }; //Phantom Bottom Hole 
	////		     = { 0.012670,0.222280, -0.104797 , 1 }; //Phantom Top Hole 
	//camera_wrt_tracker
	//		<< 0.00509314, 0.986931, 0.161062, 0.153908,
	//		0.999845, -0.00231307, -0.0174436, 0.168888,
	//		-0.0168431, 0.161126, -0.98679, -0.0937206,
	//		0, 0, 0, 1;

	//Cadaver Study-03.04
	std::vector<int> interval = { 10 };
	std::vector<float> icp_param = { 0, 1e-5,1e-6, 1e-7, 1e-8, 1e-9, 1e-10, 1e-11 };
	std::vector<std::string> params = { "0","1e5","1e6","1e7","1e8", "1e9","1e10","1e11"};	

	camera_wrt_tracker
		<< 0.00195925, 0.987534, 0.157395, 0.155608,
		0.999789, 0.00128024, -0.0204779, 0.171515,
		-0.0204241, 0.157402, -0.987323, -0.0941696,
		0, 0, 0, 1;

	////Head 1 
	//root.resize(1);
	////root[0] = "D:/CadaverStudyDay3/Rtrack/Head1/Head1_Hole4_Yaw";
	//root[0] = "D:/CadaverStudyDay3/Rtrack/Head1/Head1_Hole4_Pitch";
	////root[2] = "D:/CadaverStudyDay3/Rtrack/Head1/Head1_Hole4_Roll";
	//head_path = "D:/CadaverStudyDay3/Head-1-Post.obj";
	//tool_origin_offest.resize(1);
	//ct_tip_pos.resize(3);
	//tool_origin_offest[0] = { -0.015807, 0.000087, -0.11663, 1 }; 
	//ct_tip_pos[0] = { 2.578, -193.646, 1234.979, 1000}; // target
	//ct_tip_pos[1] = { 0.757, -194.008, 1236.614, 1000 }; // zeta 
	//ct_tip_pos[3] = { 3.08298,-194.105,1235.78, 1000 }; // button 
	//for (int _ = 0; _ < ct_tip_pos.size(); _++) {
	//	ct_tip_pos[_] /= 1000;
	//}

	////////Head 2 
	//root.resize(3);
	//root[0] = "D:/CadaverStudyDay3/Rtrack/Head2/head2_hole4_yaw";
	//root[1] = "D:/CadaverStudyDay3/Rtrack/Head2/head2_hole4_pitch";
	//root[2] = "D:/CadaverStudyDay3/Rtrack/Head2/head2_hole4_roll";
	//head_path = "D:/CadaverStudyDay3/Head-2-Post.obj";
	//tool_origin_offest.resize(3);
	//ct_tip_pos.resize(4);
	//tool_origin_offest[0] = { -0.015658, 0.000007, -0.117207, 1 };
	//tool_origin_offest[1] = { -0.015238, 0.000341, -0.116962, 1 };
	//tool_origin_offest[2] = { -0.015236, 0.000045, -0.117202, 1 };
	//ct_tip_pos[0] = { 0.482902,-214.146,1361.81,1000 };
	//ct_tip_pos[1] = { 0.522316,-214.221,1362.05,1000 };
	//ct_tip_pos[2] = { 0.501454,-214.178,1362.05,1000 };
	//ct_tip_pos[3] = { 0.521183,-214.152,1361.92,1000 };
	//for (int _ = 0; _ < ct_tip_pos.size(); _++) {
	//	ct_tip_pos[_] /= 1000;
	//}

	//////Head 3
	//root.resize(1);
	//tool_origin_offest.resize(1);
	//ct_tip_pos.resize(3);
	////root[0] = "D:/CadaverStudyDay3/Rtrack/Head3/Head3_Hole5_Yaw";
	////root[0] = "D:/CadaverStudyDay3/Rtrack/Head3/Head3_Hole5_Pitch";
	//root[0] = "D:/CadaverStudyDay3/Rtrack/Head3/Head3_Hole5_Roll";
	//head_path = "D:/CadaverStudyDay3/Head-3-Post.obj";
	//tool_origin_offest[0] = { -0.015183, 0.000352, -0.11520, 1 };
	//ct_tip_pos[0] = { 5.557, -163.735, 1202.726, 1000 }; //target 0 
	//ct_tip_pos[1] = { 5.745, -163.598, 1202.391, 1000 }; //zeta 1
	//ct_tip_pos[2] = { 6.09012,-163.787,1202.51, 1000 }; //button push 2
	//for (int _ = 0; _ < ct_tip_pos.size(); _++) {
	//	ct_tip_pos[_] /= 1000;
	//}
	//

	//Head 4 
	root.resize(3);
	root[0] = "D:/CadaverStudyDay3/Rtrack/Head4/head4_hole3_yaw";
	root[1] = "D:/CadaverStudyDay3/Rtrack/Head4/head4_hole3_pitch";
	root[2] = "D:/CadaverStudyDay3/Rtrack/Head4/head4_hole3_roll";
	head_path = "D:/CadaverStudyDay3/Head-4-Post.obj";
	tool_origin_offest.resize(1);
	ct_tip_pos.resize(3);
	tool_origin_offest[0] = { -0.015599, 0.001089, -0.114544, 1 };
	ct_tip_pos[0] = { -14.964, -186.492, 1258.010,1000 }; //target
	ct_tip_pos[1] = { -15.405, -186.457, 1258.632,1000 }; //zeta
	ct_tip_pos[2] = { -14.1738,-186.469, 1258.48, 1000 }; //ButtonPUsh
	for (int _ = 0; _ < ct_tip_pos.size(); _++) {
		ct_tip_pos[_] /= 1000;
	}

	////Head 5 
	//root.resize(1);
	//root[0] = "D:/CadaverStudyDay3/Rtrack/Head5/head5_hole1_yaw";
	////root[1] = "D:/CadaverStudyDay3/Rtrack/Head5/head5_hole1_pitch";
	////root[2] = "D:/CadaverStudyDay3/Rtrack/Head5/head5_hole1_roll";
	//head_path = "D:/CadaverStudyDay3/Head-5-Post.obj";
	//tool_origin_offest.resize(1);
	//ct_tip_pos.resize(3);
	//tool_origin_offest[0] = { -0.015147, 0.000093, -0.114288, 1 };
	//ct_tip_pos[0] = { -7.012, -195.742, 1254.549,1000 }; //target
	//ct_tip_pos[1] = { -7.163, -195.801, 1254.083,1000 }; //zeta
	//ct_tip_pos[2] = { -6.92964,-195.458,1255.51,1000 }; //button pushs
	//for (int _ = 0; _ < ct_tip_pos.size(); _++) {
	//	ct_tip_pos[_] /= 1000;
	//}


	//Generate TRE
	zeta::PointCloudPtr head_cloud(new zeta::PointCloud);
	zeta::NormalCloudPtr head_normals(new zeta::NormalCloud);
	load_head_cloud(head_path, head_cloud, head_normals);
	
	for (int k = 0; k < root.size(); k++) {
		//load timestamps and poses
		std::vector<uint64_t> time_stamps;
		std::vector<Eigen::Matrix4f> tool_pos;
		csv_reader(root[k] + "/positions.csv", time_stamps, tool_pos);
		auto p = sort_permutation(time_stamps,
			[](uint64_t const& a, uint64_t const& b) {return a < b; });
		time_stamps = apply_permutation(time_stamps, p);
		tool_pos = apply_permutation(tool_pos, p);
		for (int i = 0; i < interval.size(); i++) {
			for (int j = 0; j < icp_param.size(); j++) {
				generateTRE(root[k], head_path, time_stamps, tool_pos, head_cloud, head_normals, camera_wrt_tracker, tool_origin_offest, ct_tip_pos, interval[i], icp_param[j], params[j]);

			}
		}
	}

}


