// s2mCalculator.cpp : Defines the entry point for the application.
//

#include "s2mCalculator.h"
// C program to create loading bar 
#include <stdio.h> 
#include <windows.h> 
using namespace std;


// Function to creating loading bar 
void loadingBar(int ticks)
{
	// 0 - black background, 
	// A - Green Foreground 
	system("color 0A");

	// Initialize char for printing 
	// loading bar 
	char a = 177, b = 219;

	printf("\n\n\n\n");
	printf("\n\n\n\n\t\t\t\t\t");
	printf("Loading...\n\n");
	printf("\t\t\t\t\t");

	// Print initial loading bar 
	for (int i = 0; i < ticks; i++)
		printf("%c", a);

	// Set the cursor again starting 
	// point of loading bar 
	printf("\r");
	printf("\t\t\t\t\t");
}


inline std::vector<int>calculateSegment(zeta::PointCloud::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZ>* kdtree, int index, float searchRadius) {
	std::vector<int> queue, visited;
	//begin queue
	queue.push_back(index);
	std::cout << "calculating segment" << std::endl;
	while (queue.size() > 0) {
		//remove head from queue
		pcl::PointXYZ searchPoint = cloud->points[queue[0]];
		visited.push_back(queue[0]);
		queue.erase(queue.begin());

		//add new neighbors to queue
		std::vector<int> neighborsID;
		std::vector<float> neighborsDistance;
		if (kdtree->radiusSearch(searchPoint, searchRadius, neighborsID, neighborsDistance) > 0)
		{
			for (int i = 0; i < neighborsID.size(); ++i) {
				int current_neighbor = neighborsID[i];
				bool inqueue = std::find(queue.begin(), queue.end(), current_neighbor) != queue.end();
				bool invisited = std::find(visited.begin(), visited.end(), current_neighbor) != visited.end();
				if (!inqueue && !invisited) {
					queue.push_back(current_neighbor);
				}
			}
		}
	}
	return visited;
}

void directory_segment(std::string directory, int selected_index, float seg_rad) {
	//Load cloud and create tree 
	std::string cloud_path = directory + "/PointCloudCapture.pcd";
	zeta::PointCloudPtr cloud(new zeta::PointCloud);
	zeta::io::loadCloudFile(cloud_path, cloud);
	pcl::KdTreeFLANN<pcl::PointXYZ>* kdtree = new pcl::KdTreeFLANN<pcl::PointXYZ>;
	kdtree->setInputCloud(cloud);

	//calculate segment
	std::vector<int> auto_segment = calculateSegment(cloud, kdtree, selected_index, seg_rad);

	//copy segment into cloud
	zeta::PointCloudPtr segmented_cloud;
	segmented_cloud.reset(new zeta::PointCloud);
	segmented_cloud->is_dense = false;
	segmented_cloud->points.resize(auto_segment.size());
	segmented_cloud->height = 1;
	segmented_cloud->width = auto_segment.size();

	for (int i = 0; i < auto_segment.size(); i++) {
		segmented_cloud->points[i] = cloud->points[auto_segment[i]];
	}

	//save cloud as pcd
	std::string filename = directory + "/face_segment.pcd";
	pcl::io::savePCDFileASCII(filename, *segmented_cloud);
	std::cout << "saved cloud to: " << filename << std::endl;

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

Eigen::Matrix4f loadTransformation(std::string path){
	//read in first cell of csv and convert to matrix 
	std::vector<Eigen::Matrix4f> outvector;
	std::ifstream str(path);

	for (int i = 0; i < 1; i++) {

		std::vector<std::string>   result;
		std::string                line;
		std::getline(str, line);
		std::stringstream          lineStream(line);
		std::string                cell;

		while (std::getline(lineStream, cell, ','))
		{
			result.push_back(cell);
		}

		return matrixify(result[0]);
	}
	
}

void toVTKMat(Eigen::Matrix4f transformationMatrix, vtkSmartPointer<vtkTransform> translation) {
	
	vtkSmartPointer<vtkMatrix4x4> transfo = vtkSmartPointer<vtkMatrix4x4>::New();

	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			transfo->SetElement(i, j, transformationMatrix(i, j));
		}
	}
	translation->SetMatrix(transfo);
	//std::cout
	//	<< transformationMatrix(0, 0) << " " << transformationMatrix(0, 1) << " " << transformationMatrix(0, 2) << " " << transformationMatrix(0, 3) << std::endl
	//	<< transformationMatrix(1, 0) << " " << transformationMatrix(1, 1) << " " << transformationMatrix(1, 2) << " " << transformationMatrix(1, 3) << std::endl
	//	<< transformationMatrix(2, 0) << " " << transformationMatrix(2, 1) << " " << transformationMatrix(2, 2) << " " << transformationMatrix(2, 3) << std::endl
	//	<< transformationMatrix(3, 0) << " " << transformationMatrix(3, 1) << " " << transformationMatrix(3, 2) << " " << transformationMatrix(3, 3) << std::endl;

}

double calculateS2m(zeta::PointCloudPtr segmented_cloud, vtkSmartPointer<vtkTransform> transform, std::string directory, std::string timestamp, std::string scan, vtkSmartPointer<vtkOBJReader> readerQuery) {

	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter =
		vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter->SetTransform(transform);
	transformFilter->SetInputConnection(readerQuery->GetOutputPort());;
	transformFilter->Update();

	vtkSmartPointer<vtkPolyData> aligned_obj_data = vtkSmartPointer<vtkPolyData>::New();
	aligned_obj_data->DeepCopy(transformFilter->GetOutput());

	//calculate s2m
	std::vector<double> squared_distances = {};
	std::vector<pcl::PointXYZ> closestPoints = {};
	std::string pcdLine = "";
	double total_distance = 0;

	//Find the Closest Point to Seletected Point 
	vtkSmartPointer<vtkCellLocator> cellLocator = vtkSmartPointer<vtkCellLocator>::New();
	cellLocator->SetDataSet(aligned_obj_data);
	cellLocator->BuildLocator();
	double closestPoint[3] = {0, 0, 0};//the coordinates of the closest point will be returned here
	double selectedPoint[3] = { 0, 0, 0 };
	pcl::PointXYZ closestPointpcl;
	double closestPointDist2 = 0; //the squared distance to the closest point will be returned here
	vtkIdType cellId; //the cell id of the cell containing the closest point will be returned here
	int subId = 0; //this is rarely used (in triangle strips only, I believe)
	double sum_dists_sq = 0;
	for (int i = 0; i < segmented_cloud->points.size(); i++) {
		selectedPoint[0] = segmented_cloud->points[i].x;
		selectedPoint[1] = segmented_cloud->points[i].y;
		selectedPoint[2] = segmented_cloud->points[i].z;
		cellLocator->FindClosestPoint(selectedPoint, closestPoint, cellId, subId, closestPointDist2);
		closestPointpcl = { (float)closestPoint[0], (float)closestPoint[1], (float)closestPoint[2] };
		pcdLine += std::to_string(closestPoint[0]) + " " + std::to_string(closestPoint[1]) + " " + std::to_string(closestPoint[2]) + " " +
			std::to_string(sqrt(closestPointDist2)) + "\n";
		total_distance += sqrt(closestPointDist2);
		sum_dists_sq += closestPointDist2;
		closestPoints.push_back(closestPointpcl);
	}
	std::ofstream outfile;
	outfile.open(directory + timestamp + "/" + scan +  "_s2m.txt");
	std::cout << "printing to " << directory + timestamp + "/" + scan + "_s2m.txt" << std::endl;
	//outfile.open(directory + timestamp + "_" + scan +  "_s2m.txt");
	outfile << "FIELDS x y z dist \n";
	outfile << pcdLine.c_str();
	outfile.close();
	return sqrt(sum_dists_sq / segmented_cloud->points.size());
}

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
		if (rows != -1 && rows != 0) {
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
		}

		rows++;
	}

}

void extract_file(std::string outpath, std::string folder, std::string file_name) {
	//store copy file to folder 
	std::string infile = folder + "/" + file_name;
	std::string outfile = outpath + "/segments/" +  (folder.erase(0,outpath.size() + 1)) + "_" + file_name;

	std::cout << outfile << std::endl;
	std::filesystem::path src = infile.c_str();
	std::filesystem::path target = outfile.c_str();

	try // If you want to avoid exception handling, then use the error code overload of the following functions.
	{
		std::filesystem::copy_file(src, target, std::filesystem::copy_options::overwrite_existing);
	}
	catch (std::exception& e) // Not using fs::filesystem_error since std::bad_alloc can throw too.  
	{
		std::cout << e.what();
	}


}

void regroup(std::string root, std::string parent_folder_name, std::string file_ending="", int extra_chars = 1) {
	//root_dir = ../Bright45/segments
	//file_dir =  ../Bright45 
	std::string file_dir = root;
	file_dir = file_dir.erase(root.size() - parent_folder_name.size(), parent_folder_name.size());
	//std::cout << "second  " << root << std::endl;
	//std::cout << "file dir " << file_dir << std::endl;



	boost::filesystem::path p(root.c_str());
	for (auto& somefilepath : boost::make_iterator_range(boost::filesystem::directory_iterator(p), {})) {
		//infile = ../segment/1238721983_face_segment.pcd
		std::string infile = somefilepath.path().string();
		//std::cout << infile << std::endl;
		//below +1 for the underscore 
		std::string time_stamp = infile;
		time_stamp.erase(0, root.size()+1);
		time_stamp.erase(13, extra_chars);
 		//time_stamp.erase(0, root.size() - extra_chars);
		//std::cout << "infile: " <<  infile << std::endl;
		//std::cout << time_stamp << std::endl;
		//target = ../1238721983/face_segment.pcd
		std::string target = file_dir + time_stamp + "/" + file_ending;
		std::cout << "target " <<  target << std::endl;
		std::filesystem::copy_file(infile, target, std::filesystem::copy_options::overwrite_existing);
	}
}

//move from cropped folder to timestamp folder
void movefromCropped() {
	std::vector<std::string> roots =
	{
		//"D:/PhantomStudy/12_18_snapshots_segmentation/Dark/cropped_noneck"
		"D:/PhantomStudy/12_18_snapshots_segmentation/Baseline/cropped_noneck",
		"D:/PhantomStudy/12_18_snapshots_segmentation/Bright/cropped_noneck",
		"D:/PhantomStudy/12_18_snapshots_segmentation/Far_Left/cropped_noneck",
		"D:/PhantomStudy/12_18_snapshots_segmentation/Left/cropped_noneck",
		"D:/PhantomStudy/12_18_snapshots_segmentation/Top_Occlusion/cropped_noneck"
	};

	std::string parent_folder_name = "cropped_noneck";
	std::string file_ending = "cropped_noneck.txt";
	int extra_chars = 19;

	for (int i = 0; i < roots.size(); i++) {
		std::cout << "root " << roots[i] << std::endl;
		regroup(roots[i], parent_folder_name, file_ending, extra_chars);
	}

}

//move from timestamp folder to cropped folder
void movetoCropped() {

	std::string root = "C:\\Users\\ZImaging\\Desktop\\12_18_snapshots_segmentation\\Far_Left";

	std::cout << "Creating new directory" << std::endl;
	std::string directory_name = root + "/cropped";
	boost::filesystem::path dir(directory_name);
	if (!(boost::filesystem::exists(dir))) {
		std::cout << "Doesn't Exist" << std::endl;
		if (boost::filesystem::create_directory(dir))
			std::cout << "....Successfully Created !" << std::endl;
	}

	boost::filesystem::path p(root.c_str());
	for (auto& somefilepath : boost::make_iterator_range(boost::filesystem::directory_iterator(p), {})) {
		//infile = ../segment/1238721983_face_segment.pcd
		//below +1 for the underscore 
		std::string time_stamp = somefilepath.path().string();
		time_stamp.erase(0, time_stamp.size() - 13);
		
		//check for source file
		std::string infile = somefilepath.path().string() + "/face_segment.pcd";
		std::cout << "infile: " <<  infile << std::endl;
		std::cout << time_stamp << std::endl;

		//check for ifstream 
		std::string target = root + "/cropped/" + time_stamp + ".txt";

		//target = ../1238721983/face_segment.pcd
		std::filesystem::copy_file(infile, target, std::filesystem::copy_options::overwrite_existing);
	}

}


// calculate s2m
void dirs2m()
{
	std::vector<std::string> csv_directories = { 
		//"D:/PhantomStudy/12_18_snapshots_segmentation\\Baseline\\Alignments.csv",
		//"D:/PhantomStudy/12_18_snapshots_segmentation\\Bright\\Alignments.csv",
		//"D:/PhantomStudy/12_18_snapshots_segmentation\\Far_Left\\Alignments.csv",
		//"D:/PhantomStudy/12_18_snapshots_segmentation\\Left\\Alignments.csv",
		"D:/PhantomStudy/12_18_snapshots_segmentation\\Top_Occlusion\\Alignments.csv",
		"D:/PhantomStudy/12_18_snapshots_segmentation\\Dark\\Alignments.csv"
	
	};

	//load heads
	int num_heads = 1;
	std::string csv_name = "Alignments.csv";
	int timestamp_Col = 0;
	int RTrack_Transform_Col = 6;
	int ThreeD_Model_Transform_Col = 13;
	int CT_Transform_Col = 17;
	int MRI_Transfrom_Col = 25;
	int CT_James_Col = 21;

	std::vector<std::string> head_paths(num_heads);

	std::string ThreeDModel_obj = "C:\\Users\\ZImaging\\Desktop\\test\\3D-Model.obj";
	std::string CT_Brigham_obj =  "C:\\Users\\ZImaging\\Desktop\\test\\CT-Brigham-1.obj";
	std::string MRI_Brigham_obj = "C:\\Users\\ZImaging\\Desktop\\test\\MRI-Brigham-4.obj";
	std::string James_CT_obj =    "C:\\Users\\ZImaging\\Desktop\\test\\CT-James-1.obj";


	//head_paths[0] = ThreeDModel_obj;
	//head_paths[1] = CT_Brigham_obj;
	//head_paths[2] = MRI_Brigham_obj;
	head_paths[0] = James_CT_obj;

	//load head pointers
	std::vector<vtkSmartPointer<vtkOBJReader>> readerQuery(num_heads);
	for (int i = 0; i < num_heads; ++i) {
		readerQuery[i] = vtkSmartPointer<vtkOBJReader>::New();
		readerQuery[i]->SetFileName(head_paths[i].c_str());
		readerQuery[i]->Update();
		std::cout << "head loaded " << std::endl;
	}

	std::cout << csv_directories.size() << std::endl;
	for (int i = 0; i < csv_directories.size(); ++i) {
		//iterate over given csv, get transformation matrix and folder corresponding	
		std::vector<std::string> time_stamps;
		std::vector<vtkSmartPointer<vtkTransform>> RTrack_Alignments;
		std::vector<vtkSmartPointer<vtkTransform>> Three_D_Model_Alignments;
		std::vector<vtkSmartPointer<vtkTransform>> CT_Brigham_Alignments;
		std::vector<vtkSmartPointer<vtkTransform>> MRI_Brigham_Alignments;
		std::vector<vtkSmartPointer<vtkTransform>> CT_James_Alignments;
		
		//read matrices and timestamps from csv
		std::cout << csv_directories[i] << std::endl;
		std::ifstream str(csv_directories[i]);
		std::string                line;
		int rows = -1;

		while(std::getline(str, line)){
			std::vector<std::string>   result;
			std::stringstream          lineStream(line);
			std::string                cell;
			int linecount = 0;
			int col_num = 0;
			if (rows != -1) {
				while (std::getline(lineStream, cell, ','))
				{
					//std::cout << col_num << std::endl;
					result.push_back(cell);
					//save timestamp
					if (col_num == timestamp_Col) {
						time_stamps.push_back(result[col_num]);

					}
					//if (col_num == RTrack_Transform_Col) {
					//	RTrack_Alignments.push_back(vtkSmartPointer<vtkTransform>::New());
					//	toVTKMat(matrixify(result[col_num]), RTrack_Alignments[rows]);

					//}
					////save 3d model transform
					//if (col_num == ThreeD_Model_Transform_Col) {
					//	Three_D_Model_Alignments.push_back(vtkSmartPointer<vtkTransform>::New());
					//	toVTKMat(matrixify(result[col_num]), Three_D_Model_Alignments[rows]);

					//}
					////save CTBrig model transform
					//if (col_num == CT_Transform_Col) {
					//	CT_Brigham_Alignments.push_back(vtkSmartPointer<vtkTransform>::New());
					//	toVTKMat(matrixify(result[col_num]), CT_Brigham_Alignments[rows]);
					//}
					////save MRI model transform
					//if (col_num == MRI_Transfrom_Col) {
					//	MRI_Brigham_Alignments.push_back(vtkSmartPointer<vtkTransform>::New());
					//	toVTKMat(matrixify(result[col_num]), MRI_Brigham_Alignments[rows]);
					//}
					if (col_num == CT_James_Col) {
						CT_James_Alignments.push_back(vtkSmartPointer<vtkTransform>::New());
						toVTKMat(matrixify(result[col_num]), CT_James_Alignments[rows]);
					}

					col_num++;
				}
			}

			rows++;
		}

		//open csv for noting rms 
		//get directory name 
		std::string directory = csv_directories[i];
		directory = directory.erase(directory.size() - csv_name.size(), directory.size());
		std::string rms_filename = directory + "rms_new.csv";
		std::cout << rms_filename << std::endl;

		std::ofstream notes;
		notes.open(rms_filename, std::ios::app);
		notes << "Timestamp" << "," << "Rtrack" << "," << "Three D Model" << "," << "CT Brigham" << "," << "MRI" << "," <<  "James" << "," << std::endl;
		//loadingBar(rows);
		for (int i = 0; i < rows; ++i) {

			std::string current_dir = directory + time_stamps[i];

			//calculate s2m for each head
			//Load cloud
			//std::string segmented_scene_path = directory + time_stamps[i] + "/face_segment.pcd";
			std::string segmented_scene_path = directory + time_stamps[i] + "/cropped_noneck.txt";

			zeta::PointCloudPtr segmented_cloud(new zeta::PointCloud);
			//zeta::io::loadCloudFile(segmented_scene_path, segmented_cloud);
			txt_reader(segmented_scene_path, segmented_cloud);


			//Calculate S2m 
			//std::cout << "Calculating S2m now" << std::endl;
			//auto t0 = timeSince();
			//double RTrack_rms = calculateS2m(segmented_cloud, RTrack_Alignments[i], directory, time_stamps[i], "rtrack", readerQuery[0]);
			////auto t1 = timeSince();
			//double ThreeD_Model_rms = calculateS2m(segmented_cloud, Three_D_Model_Alignments[i], directory, time_stamps[i],"threeD", readerQuery[0]);
			////auto t2 = timeSince();
			//double CT_rms = calculateS2m(segmented_cloud, CT_Brigham_Alignments[i], directory, time_stamps[i],"ct", readerQuery[1]);
			////auto t3 = timeSince();
			//double MRI_rms = calculateS2m(segmented_cloud, MRI_Brigham_Alignments[i], directory, time_stamps[i],"mri", readerQuery[2]);
			////auto t4 = timeSince();
			double James_rms = calculateS2m(segmented_cloud, CT_James_Alignments[i], directory, time_stamps[i], "james", readerQuery[0]);
			//std::cout << std::to_string(James_rms) << std::endl;
			//auto t5 = timeSince();
			//note in csv 
			notes << 
				time_stamps[i] << "," << 
				//std::to_string(RTrack_rms) << "," << 
				//std::to_string(ThreeD_Model_rms) << "," << 
				//std::to_string(CT_rms) << "," << 
				//std::to_string(MRI_rms) << "," << 
				std::to_string(James_rms) << "," <<
				std::endl;
			//printf("%c", 219);
			std::cout << i <<" out of 221"  << std::endl;
		}
		notes.flush();
		notes.close();
	}

	return;
}

//single s2m
int singles2m() {
	//make heat map
	std::string head_path = "C:/Users/ZImaging/Desktop/test/3D-Model.obj";
	//std::string head_path = "C:/Users/ZImaging/Desktop/test/CT-Brigham-1.obj";
	//std::string head_path = "C:/Users/ZImaging/Desktop/test/CT-James-1.obj";
	//std::string head_path = "C:/Users/ZImaging/Desktop/test/MRI-Brigham-4.obj";
		

	std::string time = "1608349949956";
	std::string mat = "0.215194>-0.921034>0.324635>-0.134193|0.203775>-0.282756>-0.937294>0.219927|0.955076>0.267854>0.126836>0.563853|0>0>0>1";
	std::string segmented_scene_path = "C:/Users/ZImaging/Desktop/test/farleft_leftovers/" + time +  "/face_segment.pcd";
	std::string directory = "C:/Users/ZImaging/Desktop/test/farleft_leftovers/" ;
	std::string label = time;
	std::string scan = "ctJames";
	Eigen::Matrix4f transformationMatrix = matrixify(mat);


	//load head
	vtkSmartPointer<vtkOBJReader> readerQuery = vtkSmartPointer<vtkOBJReader>::New();
	readerQuery->SetFileName(head_path.c_str());
	readerQuery->Update();

	//Load cloud
	zeta::PointCloudPtr segmented_cloud(new zeta::PointCloud);
	zeta::io::loadCloudFile(segmented_scene_path, segmented_cloud);
	//txt_reader(segmented_scene_path, segmented_cloud);


	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	toVTKMat(transformationMatrix, transform);


	//calculate heat map
	std::cout << calculateS2m(segmented_cloud, transform, directory, label, scan, readerQuery) << std::endl;


	return 0;
}


int main() {
	dirs2m();

}

