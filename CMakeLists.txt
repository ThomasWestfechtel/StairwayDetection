cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(stair_detection)

find_package(PCL 1.7 REQUIRED)

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choosing Release build type - the fastest option."
       FORCE )
endif()

include_directories (${PCL_INCLUDE_DIRS})
include_directories ("${PROJECT_SOURCE_DIR}/include")

link_directories (${PCL_LIBRARY_DIRS})
add_definitions (${PCL_DEFINITIONS})

add_executable (stair_det src/stairDetectionMain.cpp  src/preanalysis.cpp src/regions.cpp src/segmentPatch.cpp src/regiongrowing.cpp src/voxSAC.cpp src/splitmerge.cpp src/planeshape.cpp src/Stairs.cpp src/StairVector.cpp src/recognition.cpp)
target_link_libraries (stair_det ${PCL_LIBRARIES})


project      (stair_detection)
find_package (PCL 1.7.1 REQUIRED)

#set  (project_SOURCES src/stairDetectionMain.cpp   src/regiongrowing.cpp src/preanalysis.cpp src/regiongrowing-orig.cpp src/regions.cpp src/segmentPatch.cpp src/clusteranalysis.cpp src/splitmerge.cpp src/recognition.cpp src/planeshape.cpp src/StairRegistration.cpp src/StairCoefficients.cpp src/Stairs.cpp src/StairVector.cpp src/referenceSegmentation.cpp src/voxelSAC.cpp src/RANSAC.cpp src/splitNormalEstimation.cpp)
