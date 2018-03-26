/*
 * StairVector.h
 *
 *  Created on: Oct 26, 2015
 *      Author: tom
 */

#ifndef STAIRVECTOR_H_
#define STAIRVECTOR_H_

#include "stairs/Stairs.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::Normal Normal;
typedef pcl::PointXYZRGB PointTC;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointTC> PointCloudC;

class StairVector {
public:
	StairVector();
//	virtual ~StairVector();

	std::vector<Stairs> stairVector;

	inline void push_back(Stairs input)
	{
		stairVector.push_back(input);
	}

	inline void pop_back()
	{
		stairVector.pop_back();
	}

	inline Stairs& at(int pos)
	{
		return stairVector.at(pos);
	}

	inline int size()
	{
		return stairVector.size();
	}

	inline void clear()
	{
		stairVector.clear();
	}

	inline void erase(int input)
	{
		stairVector.erase(stairVector.begin()+input);
	}

	void sort();

    regions getAllRegions();
    PointCloudC getColoredCloud(int pos = 0);
    PointCloudC getColoredParts(int pos = 0);
};

#endif /* STAIRVECTOR_H_ */
