#include "segmentviewer.h"
#include "ui_segmentviewer.h"



//helper

void saveRecording(int num_clouds, std::string directory, std::vector<zeta::PointCloudColorAPtr> clouds, std::vector<uint64_t> times, std::vector<Eigen::Matrix4f> pos) {
	
	////create new directory
	std::cout << "Creating new directory" << std::endl;
	std::string directory_name = directory;
	boost::filesystem::path dir(directory_name);
	if (!(boost::filesystem::exists(dir))) {
		std::cout << "Doesn't Exist" << std::endl;
		if (boost::filesystem::create_directory(dir))
			std::cout << "....Successfully Created !" << std::endl;
	}

	//Save Notes file 
	std::ofstream notes;
	notes.open(directory_name + "/positions.csv", std::ofstream::out);
	notes << "Timestamp, position" << std::endl;


	for (int i = 0; i < num_clouds; i++) {
		std::string filename = directory_name + "/" + std::to_string(times[i]) + ".pcd";
		pcl::io::savePCDFileASCII(filename, *clouds[i]);

		notes << times[i] << ","
			  << pos[i](0, 0) << ">" << pos[i](0, 1) << ">" << pos[i](0, 2) << ">" << pos[i](0, 3) << "|"
			  << pos[i](1, 0) << ">" << pos[i](1, 1) << ">" << pos[i](1, 2) << ">" << pos[i](1, 3) << "|"
			  << pos[i](2, 0) << ">" << pos[i](2, 1) << ">" << pos[i](2, 2) << ">" << pos[i](2, 3) << std::endl;

	}


}
void segmentViewer::resetCameraView()
{
	viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(0);
	viewer->setCameraPosition(-0.0925545, 0.461904, -1.38257, -0.169845, -0.960983, -0.218323);

	grid_actor1_->AllAxesOff();
	grid_actor1_->TopAxisVisibilityOn();
	grid_actor1_->RightAxisVisibilityOn();
	grid_actor1_->LegendVisibilityOff();

	viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->AddViewProp(grid_actor1_);

	viewer->getRenderWindow()->SetAlphaBitPlanes(1);
	viewer->getRenderWindow()->SetMultiSamples(0);
	viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->SetUseDepthPeeling(1);
	viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->SetMaximumNumberOfPeels(25);
	viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->SetOcclusionRatio(0.01);
}


void segmentViewer::startRecordingButtonPressed() {
	if (is_recording) {
		std::cout << "recording already in progress" << std::endl;
	}
	else {
		std::cout << "starting recording" << std::endl;
		is_recording = true;

	}
}

void segmentViewer::stopRecordingButtonPressed() {
	if (is_recording) {
		std::cout << "stopped recording" << std::endl;
		int num_clouds = stored_clouds.size();
		std::cout << "num stored clouds= " << num_clouds << std::endl;
		std::cout << "over " << time_stamps[num_clouds - 1] - time_stamps[0]<< "ms" << std::endl;
		std::cout << "saving recording" << std::endl;
		saveRecording(num_clouds, ui->folderNamelineEdit->text().toStdString(), stored_clouds, time_stamps, position);
		stored_clouds = {};
		time_stamps = {};
		position = {};
		std::cout << "recording saved" << std::endl;
		is_recording = false;

	}
	else {
		std::cout << "recording not in progress" << std::endl;
	}
}


void segmentViewer::visualizerSlot() {
	viewer->removePointCloud("CurrentCloud");
	//QMutexLocker locker(&cloud_capture_mtx_0);
	viewer->addPointCloud<pcl::PointXYZRGBA>(current_cloud, "CurrentCloud");
	ui->qvtkWidget->update();
	return;
}


void segmentViewer::phoxi_callback(const zeta::CloudAndNumber& cloudNumPacked){

	current_cloud = cloudNumPacked.cloud;
	//int index = cloudNumPacked.cameraNumber;
	//if (index >= 1) return;

	//QMutexLocker locker(&cloud_capture_mtx_0);
	//current_cloud->points.resize(cloudin->points.size());
	//for (int i = 0; i < cloudin->points.size(); ++i)
	//{
	//	current_cloud->points[i] = cloudin->points[i];
	//}

	if (is_recording) {
		time_stamps.push_back(timeSinceEpochMillisec());
		stored_clouds.push_back(current_cloud);
		position.push_back(chessboardTransformNdi);

		//stored_clouds[cloud_number].reset(new zeta::PointCloud);
		//stored_clouds[cloud_number]->is_dense = false;
		//stored_clouds[cloud_number]->points.resize(current_cloud->points.size());

		//for (int i = 0; i < current_cloud->size(); ++i)
		//{
		//	stored_clouds[cloud_number]->points[i].x = current_cloud->points[i].x;
		//	stored_clouds[cloud_number]->points[i].y = current_cloud->points[i].y;
		//	stored_clouds[cloud_number]->points[i].z = current_cloud->points[i].z;

		//}
		return;
	}


	return;
}


//main function
segmentViewer::segmentViewer(QWidget* parent) : QMainWindow(parent), ui(new Ui::segmentViewer)
{
	// Set up VTK App
	vtkObject::GlobalWarningDisplayOff();
	ui->setupUi(this);
	this->setWindowTitle("Auto Segment Tester");

	// Set up the QVTK Viewer
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
	ui->qvtkWidget->update();
	grid_actor1_ = vtkSmartPointer<vtkLegendScaleActor>::New();

	// Set up NDI receiving
	chessboardPast.resize(pastSize);
	ndiReceiverSocket = new QUdpSocket(this);
	ndiReceiverSocket->bind(45453, QUdpSocket::ShareAddress);
	connect(ndiReceiverSocket, &QUdpSocket::readyRead, this, &segmentViewer::processNDIBroadcast);

	//button push
	connect(ui->startRecordingButton, SIGNAL(clicked()), this, SLOT(startRecordingButtonPressed()));
	connect(ui->stopRecordingButton, SIGNAL(clicked()), this, SLOT(stopRecordingButtonPressed()));

	//Initialization
	is_recording = false;
	cloud_number = 0;
	current_cloud.reset(new zeta::PointCloudColorA);
	current_cloud->is_dense = false;

	//grabber setup 
	grabber = new zeta::PhoxiGrabber(1);
	boost::function<void(const zeta::CloudAndNumber&)> f = boost::bind(&segmentViewer::phoxi_callback, this, _1);
	grabber->registerCallback(f);
	grabber->start();

	// Set up visualization loop
	vis_timer = new QTimer();
	vis_timer->start(30);
	connect(vis_timer, SIGNAL(timeout()), this, SLOT(visualizerSlot()));

	// Update VTK Viewer after everything is set up
	resetCameraView();
	ui->qvtkWidget->update();
}
 
segmentViewer::~segmentViewer()
{

}


inline Eigen::Matrix4f strToTransMat(std::string str)
{
	Eigen::Matrix4f out;
	out.setIdentity();

	std::vector<std::string> values;
	boost::split(values, str, boost::is_any_of(",;"));

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			out(i, j) = stof(values[i * 4 + j]);
		}
	}

	return out;
}

inline Eigen::Vector4f matToQuatVec(Eigen::Matrix3f in, int pastSize)
{
	Eigen::Vector4f out;

	Eigen::Quaternionf q(in);
	out << q.w(), q.x(), q.y(), q.z();

	return out / pastSize;
}

inline Eigen::Matrix3f vecToQuatMat(Eigen::Vector4f in)
{
	Eigen::Quaternionf q(in[0], in[1], in[2], in[3]);
	return q.normalized().toRotationMatrix();
}

inline Eigen::Matrix3f quatAverage(Eigen::Matrix<float, 4, Eigen::Dynamic> allQmat)
{

	Eigen::Vector4f firstQuat = allQmat.col(0);
	Eigen::Vector4f finalQuat = allQmat.col(0);

	for (int i = 1; i < allQmat.rows(); ++i)
	{
		Eigen::Vector4f nextQuat = allQmat.col(i);
		if (firstQuat.dot(nextQuat) < 0)
		{
			finalQuat += nextQuat;
		}
		else
		{
			finalQuat -= nextQuat;
		}
	}
	//std::cout << finalQuat << std::endl;

	finalQuat = finalQuat / allQmat.rows();
	finalQuat = finalQuat.normalized();

	Eigen::Matrix3f outMat = vecToQuatMat(finalQuat);
	return(outMat);
}

inline Eigen::Matrix4f temporalAverage(std::vector< Eigen::Matrix<float, 4, 4> > pastData)
{
	Eigen::Vector3f transComponent;
	transComponent << 0, 0, 0;
	for (int i = 0; i < pastData.size(); ++i)
	{
		transComponent += pastData[i].col(3).head(3);
	}
	transComponent = transComponent / pastData.size();

	Eigen::Matrix<float, 4, Eigen::Dynamic> allQmat;
	allQmat.resize(4, pastData.size());
	for (int i = 0; i < pastData.size(); ++i)
	{
		Eigen::Vector4f rotasquat = matToQuatVec(pastData[i].block(0, 0, 3, 3), pastData.size());
		allQmat.col(i) = rotasquat;
	}

	Eigen::Matrix4f out = Eigen::Matrix4f::Identity();
	out.col(3).head(3) = transComponent;
	out.block(0, 0, 3, 3) = quatAverage(allQmat);

	return out;
}

void segmentViewer::processNDIBroadcast()
{
	while (ndiReceiverSocket->hasPendingDatagrams())
	{
		QByteArray datagram;
		datagram.resize(int(ndiReceiverSocket->pendingDatagramSize()));
		ndiReceiverSocket->readDatagram(datagram.data(), datagram.size());
		std::string messageFromServer = datagram.toStdString();

		std::vector<std::string> messageParts;
		boost::split(messageParts, messageFromServer, boost::is_any_of("/"));

		int instrumentIndex = std::stoi(messageParts[0]);
		uint64_t trackTime = std::stoull(messageParts[1]);
		if (instrumentIndex == 0)
		{
			ui->ndiConnectTextLabel->setText("NDI Connected");

			Eigen::Matrix4f newChessboard = strToTransMat(messageParts[2]);

			for (int i = 1; i < pastSize; ++i)
				chessboardPast[i] = chessboardPast[i - 1];
			chessboardPast[0] = newChessboard;

			chessboardTransformNdi = temporalAverage(chessboardPast);
		}
	}
}