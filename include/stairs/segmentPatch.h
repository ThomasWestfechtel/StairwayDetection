/*
 * segmentPatch.h
 *
 *  Created on: Nov 25, 2014
 *      Author: tom
 */

#ifndef SEGMENTPATCH_H_
#define SEGMENTPATCH_H_

#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <deque>
#include <map>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::Normal Normal;
typedef pcl::PointXYZRGB PointTC;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointTC> PointCloudC;

//using namespace std;

class segmentPatch {
public:
	PointCloudT segmentCloud;
	NormalCloud normalCloud;
	pcl::PointIndices::Ptr segmentIndices;

	PointCloudT borderPoints;

	segmentPatch()
	: segmentIndices (new pcl::PointIndices)
	{
	}

	~segmentPatch()
	{
		segmentCloud.clear();
		normalCloud.clear();
		borderPoints.clear();
	}

//	segmentPatch(const segmentPatch &rv)
//	{
//		std::cout<<"Segment Patch Copy constructor called!"<<std::endl;
//	}

	Eigen::Vector4f segmentCoefficient;
	Eigen::Vector4f segmentCentroid;
	Eigen::Matrix3f segmentCovariance;

	Eigen::Matrix3f eigen_vectors;
	Eigen::Vector3f eigen_values;

	Eigen::Vector3f dimensions;
	Eigen::Vector3f middleExtensions;

	Eigen::Vector3i roomPos;
	int voxSize;

    float maxHeight;

    Eigen::Vector2f depth;
    Eigen::Vector2f depthVariance;
    Eigen::Vector2f width;
    Eigen::Vector2f height;

	float curvature;
    int recongitionRefs;
	int segmentLabel;

    std::vector<int> partner;
	std::vector<int> globalIndices;

	void analyse();
	void analyse(PointCloudT::Ptr wholeCloud);

	void createBorder();

	void initializeExtensions();

    void getMaxHeight();
    void getHeight();
    void getTreadDepth(Eigen::Vector3f stairDir);
    void getWidth(Eigen::Vector3f pos, Eigen::Vector3f dirVec);

    void getRailHeight(Eigen::Vector3f extensionDir, Eigen::Vector3f refPoint, Eigen::Vector2f& result);
    void getRailLength(Eigen::Vector3f extensionDir, Eigen::Vector3f refPoint, Eigen::Vector2f& result);
    void getRailWidth(Eigen::Vector3f extensionDir, Eigen::Vector3f refPoint, Eigen::Vector2f& result);

    void writeSegment(std::ofstream& writeFile);
    void readSegment(std::ifstream& loadFile);
	//void eigenAnalysis(Eigen::Matrix3f& mat, Eigen::Matrix3f& eigenvector, Eigen::Vector3f& eigenvalues);
};
#endif /* SEGMENTPATCH_H_ */
