#ifndef PREANALYSIS_H
#define PREANALYSIS_H

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/filters/statistical_outlier_removal.h>

//#include <stairs/normal_3d.h>
#include <stairs/normal_3d_omp.h>

#include <pcl/common/time.h>

#include <pcl/common/transforms.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::Normal Normal;
typedef pcl::PointXYZRGB PointTC;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointTC> PointCloudC;

typedef std::vector<int> Indices;

typedef typename pcl::octree::OctreePointCloud<pcl::PointXYZ> OctreeT;
typedef typename OctreeT::LeafContainer LeafContainer;

class Preanalysis {
public:

    Preanalysis();
//    ~Preanalysis();

    void limitPC();
    void downsample();

    void normalEstimation();
    void ghostPointFilter();
    void floorExtraction();
    void run(PointCloudT::Ptr& input, NormalCloud::Ptr& normal, PointCloudC& colMap, PointCloudT& floorPoints);

    inline
    void setDsResolution(double resolution)
    {
        dsResolution = resolution;
    }

    inline void setRobotAngle(double angle)
    {
    	robAngle = angle/180*M_PI;
    }

    inline void setGpActive(bool input)
    {
        gpFlag = input;
    }

    inline void setDsFlag(bool input)
    {
    	dsFlag = input;
    }

    inline
    void setNeSearch(int method)
    {
    	neNeighMethod = method;
    }

    inline
    void setSearchNeighbours(int searchNeighbors)
    {
        neSearchNeighbours = searchNeighbors;
    }

    inline
    void setSearchRadius(double searchRadius)
    {
        neSearchRadius = searchRadius;
    }

    inline
    void setGpAngle(double angle)
    {
        gpAngle = angle;
    }

    inline
    void setFsActive(bool active)
    {
        fsActive = active;
    }

    inline
    void setFsAngle(double angle)
    {
        fsAngle = angle;
    }

    inline
    void setPfActive(bool active)
    {
        pfActive = active;
    }

    inline
    void setPfAngle(double angle)
    {
        pfAngle = angle;
    }

    inline
    void setFsRange(double range)
    {
        fsRange = range;
    }

    inline void setDsMethod(bool input)
    {
    	dsMethod = input;
    }

    PointCloudT::Ptr inputCloud;
    PointCloudT::Ptr pc;
    NormalCloud::Ptr normal_cloud;
    PointCloudT floorPC;
    NormalCloud floorNormal;


    double rob_x;
    double rob_y;
    double rob_z;
    double robAngle;

    double neTime;
    int neMethod;

    inline void setNeMethod(int value)
    {
    	neMethod = value;
    }

    inline void getNeTime(double& time)
    {
    	time = neTime;
    }

    bool dsFlag;
    bool dsMethod;
    bool gpFlag;

    double dsResolution;
    int neNeighMethod;
    int neSearchNeighbours;
    double neSearchRadius;
    double gpAngle;
    bool fsActive;
    double fsAngle;
    double fsRange;

    bool pfActive;
    double pfAngle;

};

#endif // PREANALYSIS

