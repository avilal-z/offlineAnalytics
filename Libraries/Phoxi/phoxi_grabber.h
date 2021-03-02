#define NOMINMAX
#include <iostream>
#include <string>
#include <vector>
#include <windows.h>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#define PHOXI_PCL_SUPPORT
#include "PhoXi.h"

namespace zeta
{
	struct CloudAndNumber {
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
		int cameraNumber;
	};

	class PhoxiGrabber : public pcl::Grabber
	{
	public:
		PhoxiGrabber(int numCameras);
		virtual ~PhoxiGrabber();
		virtual void start();
		virtual void stop();
		virtual bool isRunning() const;
		virtual std::string getName() const;
		virtual float getFramesPerSecond() const;

	protected:
		typedef void (signal_Phoxi_cloud_and_number)(const CloudAndNumber&);
		boost::signals2::signal<signal_Phoxi_cloud_and_number>* signal_PointXYZRGBA;

		boost::thread thread;
		mutable boost::mutex mutex;

		void threadFunction();

		bool quit;
		bool running;

		int numCameras;
		pho::api::PPhoXi PhoXiDevice;
	};
}