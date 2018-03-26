/*
 * regions.h
 *
 *  Created on: Feb 2, 2015
 *      Author: tom
 */

#ifndef REGIONS_H_
#define REGIONS_H_

#include <stairs/segmentPatch.h>
#include <deque>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <pcl/common/time.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::Normal Normal;
typedef pcl::PointXYZRGB PointTC;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointTC> PointCloudC;

//using namespace std;

class regions {
public:
	std::vector<segmentPatch> regs;
	PointCloudT centerCloud;
	PointCloudT floorCloud;

    PointCloudC getColoredCloud();
    PointCloudC getNormalMap();

    void sortByRefs();

    void generateCenterCloud();

    void createBorders();
    void showBorders(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);

    void saveRegion (std::string savePath);
    void loadRegion (std::string loadPath);

	inline void combine(regions r1, regions r2)
	{
		regs = r1.regs;
		for(int curIdx = 0; curIdx < r2.size(); curIdx++)
		{
			regs.push_back(r2.at(curIdx));
		}
	}

	inline void add(regions r2)
	{
		int labelId = regs.size();
		for(int curIdx = 0; curIdx < r2.size(); curIdx++)
		{
			regs.push_back(r2.at(curIdx));
			regs.at(labelId).segmentLabel=labelId;
			labelId++;
		}
	}

    regions()
    {
    }

    ~regions()
    {
    	regs.clear();
    	centerCloud.clear();
    	floorCloud.clear();
    }

    inline void setFloorCloud(PointCloudT inputCloud)
    {
    	floorCloud = inputCloud;
    }

    inline int size() const
    {
        return regs.size();
    }

    inline void clear()
    {
        regs.clear();
        centerCloud.clear();
        floorCloud.clear();
    }

    inline segmentPatch& at(int pos)
    {
        return(regs.at(pos));
    }

    inline void push_back(segmentPatch seg)
    {
        regs.push_back(seg);
    }

    inline void analyse()
    {
        for(int regCount =0; regCount<regs.size(); regCount++)
        {
            regs.at(regCount).analyse();
        }
    }

    inline void getExtensions()
    {
        for(int regCount =0; regCount<regs.size(); regCount++)
        {
            regs.at(regCount).initializeExtensions();
        }
    }

};

inline bool compareRefs (segmentPatch first, segmentPatch second)
{
  return (first.recongitionRefs > second.recongitionRefs);
}


#endif /* REGIONS_H_ */
