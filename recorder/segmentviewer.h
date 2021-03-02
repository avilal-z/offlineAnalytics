#ifndef SYLVESTERVIEWER_H
#define SYLVESTERVIEWER_H

// Standard Includes
#include <iostream>
#include <string>
#include <thread>
#include <mutex>

// Boost Includes
#include <boost/algorithm/string.hpp>

// Eigen
#include <Eigen/Dense>

// Qt
#include <QMainWindow>
#include <QMutexLocker>
#include <QObject>
#include <QKeyEvent>
#include <QFuture>
#include <QtConcurrent/QtConcurrent>
#include <QClipboard>
#include <QApplication>
#include <QStyleFactory>
#include <QAction>
#include <QCheckBox>
#include <QMessageBox>
#include <QTableWidgetItem>
#include <QtNetwork>
#include <QFileDialog>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni2_device_manager.h>
#include <pcl/io/openni2/openni2_metadata_wrapper.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>
#include <vtkLegendScaleActor.h>
#include <QVTKOpenGLWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkImageChangeInformation.h>
#include <vtkImageMapper.h>
#include <vtkCellLocator.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

// Zeta
#include <zeta/zeta_io.h>
#include <zeta/zeta_types.h>
#include <Libraries/z/ZetaAligner.h>
#include "Libraries/Phoxi/phoxi_grabber.h"

typedef pcl::PointXYZRGBA PointColorKinect;
typedef pcl::PointCloud<PointColorKinect> PointCloudKinect;

namespace Ui {
	class segmentViewer;
}

class segmentViewer : public QMainWindow
{
	Q_OBJECT

public:
	explicit segmentViewer(QWidget* parent = 0);
	~segmentViewer();

public slots:
	void processNDIBroadcast();

protected:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	vtkSmartPointer<vtkLegendScaleActor> grid_actor1_;

	QMutex ui_values_mtx;
	bool is_recording;
	zeta::PhoxiGrabber* grabber;
	QMutex cloud_capture_mtx_0;
	QTimer* vis_timer;

	zeta::PointCloudColorAPtr current_cloud;
	int cloud_number;
	std::vector<zeta::PointCloudColorAPtr> stored_clouds;
	std::vector<uint64_t> time_stamps;
	std::vector<Eigen::Matrix4f> position;

	// tracking stuff
	QUdpSocket* ndiReceiverSocket;
	int pastSize = 10;
	std::vector<Eigen::Matrix4f> chessboardPast;
	Eigen::Matrix4f chessboardTransformNdi = Eigen::Matrix4f::Zero();


private:
	Ui::segmentViewer* ui;
	void phoxi_callback(const zeta::CloudAndNumber& cloudNumPacked);
	void resetCameraView();
	void visCloud(std::string cloud_name, zeta::PointCloudColorAPtr cloud);


private Q_SLOTS:
	void visualizerSlot();
	void startRecordingButtonPressed();
	void stopRecordingButtonPressed();
};

#endif // SYLVESTERVIEWER_H