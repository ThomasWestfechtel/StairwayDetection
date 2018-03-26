#ifndef REGIONGROWING
#define REGIONGROWING

#include <pcl/point_types.h>
#include <pcl/common/time.h>

#include <stairs/region_growing.h>

#include <stairs/regions.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::Normal Normal;
typedef pcl::PointXYZRGB PointTC;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointTC> PointCloudC;

class RegionGrowing{
public:

	RegionGrowing();
//    ~RegionGrowing();

    PointCloudT::Ptr inputCloud;
    NormalCloud::Ptr normalCloud;

    PointCloudC::Ptr cloud_segmented;

//    regions segmentList;

    int minClustSize;
    int noNeigh;
    bool smoothFlag;
    double smoothThresh;
    bool resFlag;
    double resThresh;
    bool curvFlag;
    double curvThresh;
    bool updateFlag;
    bool pointUpdateFlag;
    int updateInterval;

    inline void setPointUpdateFlag(bool value)
    {
        pointUpdateFlag = value;
    }

    inline void setClusterSize(int value)
    {
        minClustSize=value;
    }

    inline void setNoNeigh(int value)
    {
        noNeigh=value;
    }

    inline void setSmoothFlag(bool value)
    {
        smoothFlag=value;
    }

    inline void setSmoothTresh(double value)
    {
        smoothThresh=value;
    }

    inline void setResFlag(bool value)
    {
        resFlag=value;
    }

    inline void setResTresh(double value)
    {
        resThresh=value;
    }

    inline void setCurvFlag(bool value)
    {
        curvFlag=value;
    }

    inline void setCurvThresh(double value)
    {
        curvThresh = value;
    }

    inline void setUpdateFlag(bool value)
    {
        updateFlag=value;
    }
    inline void setUpdateInterval (int value)
    {
        updateInterval=value;
    }

    void run(regions& output);
//    void analyzePrimitives();
//    void filter(regions& segmentListFilt);

    inline void setInputCloud(PointCloudT::Ptr input)
    {
        inputCloud=input;
    }
    inline void setNormalCloud(NormalCloud::Ptr normal)
    {
        normalCloud=normal;
    }

};
#endif // REGIONGROWING

