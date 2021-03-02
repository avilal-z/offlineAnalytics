cmake_minimum_required( VERSION 3.8 )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Create Project
project(segmenter VERSION 0.0.1 LANGUAGES CXX CUDA)

option(CMAKE_USE_WIN32_THREADS_INIT "using WIN32 threads" ON)

#Find Packages
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()



find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package( VTK REQUIRED )


set( QTDIR $ENV{QTDIR} )
set( CMAKE_PREFIX_PATH ${QTDIR} )
find_package( Qt5Widgets REQUIRED )
find_package( Qt5MultimediaWidgets REQUIRED)

find_package(CUDA)
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
add_definitions(${CUDA_DEFINITIONS})

# Set Automatic MOC
set( CMAKE_INCLUDE_CURRENT_DIR ON )
set( CMAKE_AUTOMOC ON )
set( CMAKE_AUTORCC ON )
set( CMAKE_AUTOUIC ON )

include_directories("C://Program Files//NVIDIA GPU Computing Toolkit//CUDA//v10.1//include")
include_directories("C:\\Users\\Zimaging\\Dev\\zeta-api\\out\\install\\x64-Release\\include")
include_directories("C:\\Users\\Zimaging\\Dev\\zeta-api\\out\\build\\x64-Release\\zeta-lab")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}\\Libraries\\Phoxi\\include")

#construct sac gpu lib
add_library(sacgpu  
	"Libraries/z/zeta-ransac.cu" 
	"Libraries/z/zeta-ransac.h" 	
)
set_property(TARGET sacgpu PROPERTY CUDA_RESOLVE_DEVICE_SYMBOLS ON)
set_target_properties(sacgpu PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(sacgpu
	"C://Program Files//NVIDIA GPU Computing Toolkit//CUDA//v10.1//lib//x64//cuda.lib"
	"C://Program Files//NVIDIA GPU Computing Toolkit//CUDA//v10.1//lib//x64//cublas.lib"

	${PCL_LIBRARIES}
	${CUDA_LIBRARIES}
)
target_compile_options(sacgpu PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
 -lineinfo
>) 

#create library for gpu func
add_library(gpuKNN  
	"${CMAKE_CURRENT_SOURCE_DIR}/Libraries/z/gpuKNN/zeta_cuda_index.cu"
	"${CMAKE_CURRENT_SOURCE_DIR}/Libraries/z/gpuKNN/zeta_cuda_index.h"
	"${CMAKE_CURRENT_SOURCE_DIR}/Libraries/z/gpuKNN/zeta_cuda_heap.h"
	"${CMAKE_CURRENT_SOURCE_DIR}/Libraries/z/gpuKNN/zeta_cuda_result_set.h"

)
set_property(TARGET gpuKNN PROPERTY CUDA_RESOLVE_DEVICE_SYMBOLS ON)
set_target_properties(gpuKNN PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(gpuKNN
	"C://Program Files//NVIDIA GPU Computing Toolkit//CUDA//v10.1//lib//x64//cuda.lib"
	"C://Program Files//NVIDIA GPU Computing Toolkit//CUDA//v10.1//lib//x64//cublas.lib"
	${CUDA_LIBRARIES}
)
qt5_wrap_ui( UI_HEADER segmentviewer.ui )

add_executable(recorder 
	"Libraries/Phoxi/phoxi_grabber.h" "Libraries/Phoxi/phoxi_grabber.cpp"
	"recorder/segmenter.cpp" "recorder/segmentviewer.h" "recorder/segmentviewer.cpp" "recorder/segmentviewer.ui"
	 "Libraries/z/zeta-ransac.h" "Libraries/z/zeta-features.hpp" "Libraries/z/ZetaAligner.h"
)

target_link_libraries (recorder 
	${QT_LIBRARIES} Qt5::Widgets  Qt5::MultimediaWidgets
	"${CMAKE_CURRENT_SOURCE_DIR}\\Libraries\\z\\zeta.lib"
	${CMAKE_CURRENT_SOURCE_DIR}/Libraries/Phoxi/lib/PhoXi_API_msvc14_Release_1.2.14.lib
	${VTK_LIBRARIES} ${QT_LIBRARIES} Qt5::Widgets
	${PCL_LIBRARIES}
	"C://Program Files//NVIDIA GPU Computing Toolkit//CUDA//v10.1//lib//x64//cuda.lib"
	sacgpu
)


add_executable(rtrackTRE 
	"include/helpers.hpp" "rtrackTRE/rtackTRE.cpp"
	"Libraries/z/gpuKNN/zeta_cuda_index.h" "Libraries/z/gpuKNN/zeta-gpuICP.hpp"
	"Libraries/z/zeta-ransac.h" "Libraries/z/zeta-features.hpp" "Libraries/z/ZetaAligner.h"

)

target_link_libraries (rtrackTRE 
	"${CMAKE_CURRENT_SOURCE_DIR}\\Libraries\\z\\zeta.lib"
	${VTK_LIBRARIES} 
	${PCL_LIBRARIES}
	"C://Program Files//NVIDIA GPU Computing Toolkit//CUDA//v10.1//lib//x64//cuda.lib"
	sacgpu
	gpuKNN
)

add_executable(rtrackSRE 
	"include/helpers.hpp" "rtrackSRE/rtrackSRE.cpp"
	"Libraries/z/gpuKNN/zeta_cuda_index.h" "Libraries/z/gpuKNN/zeta-gpuICP.hpp"
	"Libraries/z/zeta-ransac.h" "Libraries/z/zeta-features.hpp" "Libraries/z/ZetaAligner.h"

)

target_link_libraries (rtrackSRE 
	"${CMAKE_CURRENT_SOURCE_DIR}\\Libraries\\z\\zeta.lib"
	${VTK_LIBRARIES} 
	${PCL_LIBRARIES}
	"C://Program Files//NVIDIA GPU Computing Toolkit//CUDA//v10.1//lib//x64//cuda.lib"
	sacgpu
	gpuKNN
)


install(TARGETS recorder
        RUNTIME DESTINATION bin)
install(TARGETS rtrackTRE
        RUNTIME DESTINATION bin)