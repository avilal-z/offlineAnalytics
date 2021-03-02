#include "phoxi_grabber.h"

void printDeviceInfo(const pho::api::PhoXiDeviceInformation& DeviceInfo)
{
    std::cout << "  Name:                    " << DeviceInfo.Name << std::endl;
    std::cout << "  Hardware Identification: " << DeviceInfo.HWIdentification
        << std::endl;
    std::cout << "  Type:                    " << std::string(DeviceInfo.Type)
        << std::endl;
    std::cout << "  Firmware version:        " << DeviceInfo.FirmwareVersion
        << std::endl;
    std::cout << "  Status:                  "
        << (DeviceInfo.Status.Attached
            ? "Attached to PhoXi Control. "
            : "Not Attached to PhoXi Control. ")
        << (DeviceInfo.Status.Ready ? "Ready to connect" : "Occupied")
        << std::endl
        << std::endl;
}

void printDeviceInfoList(const std::vector<pho::api::PhoXiDeviceInformation>& DeviceList)
{
    for (std::size_t i = 0; i < DeviceList.size(); ++i)
    {
        std::cout << "Device: " << i << std::endl;
        printDeviceInfo(DeviceList[i]);
    }
}

zeta::PhoxiGrabber::PhoxiGrabber(int numCameras)
	: running(false)
	, quit(false)
	, signal_PointXYZRGBA(nullptr)
{
	this->numCameras = numCameras;
    signal_PointXYZRGBA = createSignal<signal_Phoxi_cloud_and_number>();

    pho::api::PhoXiFactory Factory;

    // Check if the PhoXi Control Software is running
    if (!Factory.isPhoXiControlRunning())
    {
        std::cout << "PhoXi Control Software is not running" << std::endl;
        return;
        //throw std::runtime_error("PhoXi Control Software is not running");
    }

    // Get List of available devices on the network
    std::vector<pho::api::PhoXiDeviceInformation> DeviceList = Factory.GetDeviceList();
    if (DeviceList.empty())
    {
        std::cout << "PhoXi Factory has found 0 devices" << std::endl;
        return;
        //throw std::runtime_error("PhoXi Factory has found 0 devices");
    }
    printDeviceInfoList(DeviceList);

    // Try to connect device opened in PhoXi Control, if any
    PhoXiDevice = Factory.CreateAndConnectFirstAttached();
    if (PhoXiDevice)
    {
        std::cout << "You have already PhoXi device opened in PhoXi Control, "
            "the API is connected to device: "
            << (std::string)PhoXiDevice->HardwareIdentification
            << std::endl;
    }
    else
    {
        std::cout << "You have no PhoXi device opened in PhoXi Control, the API ";
        for (size_t i = 0; i < DeviceList.size(); i++)
        {
            std::cout << " will try to connect to ..."
                << DeviceList.at(i).HWIdentification << std::endl;
            // wait 5 second for scanner became ready
            PhoXiDevice = Factory.CreateAndConnect(
                DeviceList.at(i).HWIdentification, 5000);
            if (PhoXiDevice)
            {
                std::cout << "succesfully connected" << std::endl;
                break;
            }
            if (i == DeviceList.size() - 1)
            {
                std::cout << "Can not connect to any device" << std::endl;
                throw std::runtime_error("Can not connect to any device");
            }
        }
    }

    // Check if device was created
    if (!PhoXiDevice)
    {
        std::cout << "Your device was not created!" << std::endl;
        throw std::runtime_error("Your device was not created!");
    }

    // Check if device is connected
    if (!PhoXiDevice->isConnected())
    {
        std::cout << "Your device is not connected" << std::endl;
        throw std::runtime_error("Your device is not connected");
    }

    if (PhoXiDevice->isAcquiring())
    {
        // Stop acquisition to change trigger mode
        PhoXiDevice->StopAcquisition();
    }

    PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
    std::cout << "Software trigger mode was set" << std::endl;
    PhoXiDevice->ClearBuffer();

    // PhoXi Basic Capturing Settings here
    //PhoXiDevice->CapturingSettings->ShutterMultiplier = 1;
    //PhoXiDevice->CapturingSettings->ScanMultiplier = 1;
    //PhoXiDevice->CapturingSettings->CameraOnlyMode = false;
    std::vector<pho::api::PhoXiCapturingMode> SupportedCapturingModes = PhoXiDevice->SupportedCapturingModes; 
	
    // For PhoXi // 0 is 2064x1544, 1 is 1032x772
    //PhoXiDevice->CapturingMode = SupportedCapturingModes[0];

    // For MotionCam // 0 is 1120x800, 1 is 1680x1200
    //PhoXiDevice->CapturingMode = SupportedCapturingModes[1];

    // PhoXi Advanced Capturing Settings
    //PhoXiDevice->CapturingSettings->AmbientLightSuppression = false;
    //PhoXiDevice->CapturingSettings->CodingStrategy = "Interreflections";
    //PhoXiDevice->CapturingSettings->CodingQuality = "High";
    //PhoXiDevice->CapturingSettings->TextureSource = "LED";
    //std::vector<double> SinglePatternExposures = PhoXiDevice->SupportedSinglePatternExposures; // 10.24 / 14.336 / 20.48 / 24.576 / 30.72 / 34.816 / 40.96 / 49.152 / 75.776 / 79.872 / 90.112 / 100.352
    //PhoXiDevice->CapturingSettings->SinglePatternExposure = SinglePatternExposures[3];
    
    // Use this for PhoXi
    //PhoXiDevice->CapturingSettings->MaximumFPS = 0;
    //PhoXiDevice->CapturingSettings->LaserPower = 4095;

    // Use this for MotionCam
    //PhoXiDevice->CapturingSettings->MaximumFPS = 20;
    //PhoXiDevice->CapturingSettings->LaserPower = 4095;



    // PhoXi Point Cloud Processing Settings
    //PhoXiDevice->ProcessingSettings->Confidence = 3.0; // Max inaccuracy (in mm)
    //PhoXiDevice->ProcessingSettings->CalibrationVolumeOnly = true;
    //PhoXiDevice->ProcessingSettings->SurfaceSmoothness = "Normal";
    //PhoXiDevice->ProcessingSettings->NormalsEstimationRadius = 2;

    // PhoXi Normal Angle Settings
    //PhoXiDevice->ProcessingSettings->NormalAngle.MaxCameraAngle = 90;
    //PhoXiDevice->ProcessingSettings->NormalAngle.MaxProjectorAngle = 90;
    //PhoXiDevice->ProcessingSettings->NormalAngle.MinHalfwayAngle = 0;
    //PhoXiDevice->ProcessingSettings->NormalAngle.MaxHalfwayAngle = 90;

    PhoXiDevice->CoordinatesSettings->CoordinateSpace = "CameraSpace";

    PhoXiDevice->StartAcquisition();
    if (!PhoXiDevice->isAcquiring())
    {
        std::cout << "Your device could not start acquisition!" << std::endl;
        throw std::runtime_error("Your device could not start acquisition!");
    }
	//std::cout << "Phoxi Mximum fps: " << PhoXiDevice->CapturingSettings->MaximumFPS << std::endl;
	//std::cout << "Phoxi Laser Power: " << PhoXiDevice->CapturingSettings->LaserPower << std::endl;
	//std::cout << "Phoxi Resolution Width: " << PhoXiDevice->Resolution->Width << std::endl;
	//std::cout << "Phoxi Resolution Height: " << PhoXiDevice->Resolution->Height << std::endl;


}

zeta::PhoxiGrabber::~PhoxiGrabber()
{
	stop();

	disconnect_all_slots<signal_Phoxi_cloud_and_number>();

	thread.join();
}

void zeta::PhoxiGrabber::stop()
{
    boost::unique_lock<boost::mutex> lock(mutex);

    std::cout << "Stopping PhoXi Acquisition" << std::endl;
    PhoXiDevice->StopAcquisition();
    std::cout << "Disconnecting PhoXi" << std::endl;
    PhoXiDevice->Disconnect();

    quit = true;
    running = false;

    lock.unlock();
}

bool zeta::PhoxiGrabber::isRunning() const
{
    boost::unique_lock<boost::mutex> lock(mutex);

    return running;

    lock.unlock();
}

std::string zeta::PhoxiGrabber::getName() const
{
    return std::string("PhoxiGrabber");
}

float zeta::PhoxiGrabber::getFramesPerSecond() const
{
    return 1.0f;
}

void zeta::PhoxiGrabber::start()
{
    running = true;
    thread = boost::thread(&PhoxiGrabber::threadFunction, this);
}

void zeta::PhoxiGrabber::threadFunction()
{
    while (!quit)
    {
        if (!PhoXiDevice)
            continue;

        boost::unique_lock<boost::mutex> lock(mutex);

        // std::cout << "Triggering the frame" << std::endl;
        int FrameID = PhoXiDevice->TriggerFrame();
        if (FrameID < 0)
        {
            std::cout << "Trigger was unsuccessful!" << std::endl;
            continue;
        }
        else
        {
            // std::cout << "Frame was triggered, Frame Id: " << FrameID << std::endl;
        }

        // std::cout << "Waiting for frame " << std::endl;
        pho::api::PFrame Frame = PhoXiDevice->GetSpecificFrame(
            FrameID, pho::api::PhoXiTimeout::Infinity);

        pcl::PointCloud<pcl::PointXYZRGBNormal> PhoxiPCLCloud;

        if (Frame)
        {
            Frame->ConvertTo(PhoxiPCLCloud);
        }
        else
        {
            std::cout << "Failed to retrieve the frame!" << std::endl;
            continue;
        }

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGBA>);
        cloud->points.resize((int64_t)PhoxiPCLCloud.width * (int64_t)PhoxiPCLCloud.height);
        cloud->width = PhoxiPCLCloud.width;
        cloud->height = PhoxiPCLCloud.height;
        cloud->is_dense = false;

        for (int i = 0; i < PhoxiPCLCloud.size(); ++i)
        {
            cloud->points[i].x = PhoxiPCLCloud.points[i].x / 1000.0f;
            cloud->points[i].y = PhoxiPCLCloud.points[i].y / 1000.0f;
            cloud->points[i].z = PhoxiPCLCloud.points[i].z / 1000.0f;

            cloud->points[i].r = PhoxiPCLCloud.points[i].r;
            cloud->points[i].g = PhoxiPCLCloud.points[i].g;
            cloud->points[i].b = PhoxiPCLCloud.points[i].b;
            cloud->points[i].a = 255.0f;
        }

        pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
        vg.setLeafSize(0.001, 0.001, 0.001);
        vg.setInputCloud(cloud);
        vg.filter(*cloud);

        lock.unlock();
        signal_PointXYZRGBA->operator()({cloud, 0});
    }
}