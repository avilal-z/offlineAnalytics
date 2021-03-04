// s2mCalculator.h : Include file for standard system include files,
// or project specific include files.
#pragma once
// Standard Includes
#include <iostream>
#include <string>
#include <filesystem>

// Boost Includes 
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

//VTK 
#include <vtkCellLocator.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

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

// Eigen
#include <Eigen/Dense>

//Zeta
#include <zeta/zeta_io.h>
#include <zeta/zeta_metrics.h>
#include <zeta/zeta_sampling.h>

inline uint64_t timeSince() {
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}