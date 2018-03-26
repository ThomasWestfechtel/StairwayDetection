/*
 * Stairs.h
 *
 *  Created on: Oct 26, 2015
 *      Author: tom
 */

#ifndef STAIRS_H_
#define STAIRS_H_

#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <vector>

#include "stairs/regions.h"

#include <iostream>


typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::Normal Normal;
typedef pcl::PointXYZRGB PointTC;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointTC> PointCloudC;

class Stairs {
public:
	Stairs();
//	virtual ~Stairs();

	Eigen::Vector3f pos;
	Eigen::Vector3f dir;
	float width;
	int anchPoint;

	int stairCount;
	int stairOffset;

	bool isCircular;
	float widthOff;
	float angleDiff;
	bool clockwise;

	float accuracy;
	Eigen::Vector3f stairScore;

	inline Eigen::Matrix<float,7,1> coeffVec()
	{
		Eigen::Matrix<float,7,1> retMat;
		retMat.head(3)=pos;
		retMat[3] = width;
		retMat.tail(3)=dir;
		return retMat;
	}

//	ostream& operator<<(ostream& os);

    PointCloudT stairRiseCloud;
    PointCloudT stairTreadCloud;
    PointCloudT stairRailCloud;

    regions stairParts;
    regions stairTreads;
    regions stairRises;
    regions stairRail;

    std::vector<int> planeLabels;

    void getColoredParts(PointCloudC& output);

    inline segmentPatch at(int pos)
    {
    	return stairParts.at(pos);
    }

    inline int size() const
    {
    	return stairParts.size();
    }

	friend std::ostream& operator<<(std::ostream& os, Stairs& sc);

	inline bool operator<(Stairs comp) const
	{
//		if((stairTreads.size() + stairRises.size()) == (comp.stairTreads.size() + comp.stairRises.size()))
//			return (stairParts.size() > comp.stairParts.size());
//		else return ((stairTreads.size() + stairRises.size()) > (comp.stairTreads.size() + comp.stairRises.size()));
		return ((stairTreads.size() + stairRises.size())*accuracy > (comp.stairTreads.size() + comp.stairRises.size())*comp.accuracy);
	}
};

#endif /* STAIRS_H_ */
