#ifndef SPLITMERGE
#define SPLITMERGE

#endif // SPLITMERGE

#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/common/time.h>

#include <boost/multi_array.hpp>
#include <cassert>

#include <stairs/segmentPatch.h>
#include <stairs/regions.h>

#include <deque>

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::Normal Normal;
typedef pcl::PointXYZRGB PointTC;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointTC> PointCloudC;

typedef std::vector<int> Indices;

typedef pcl::octree::OctreePointCloud<PointT> OctreeT;
typedef typename OctreeT::LeafContainer LeafContainer;

typedef boost::shared_ptr <std::vector<int> > IndicesPtr;



class splitMerge {
public:

    int uppLimit;

    std::map<std::vector<int>,int> occupMap;
    std::map<std::vector<int>,int>::iterator occupMapIter;

    std::vector<segmentPatch> priComp;
    std::vector<segmentPatch> mergeComp;
    PointCloudT::Ptr inputCloud;
    PointCloudT::Ptr remCloud;
    NormalCloud::Ptr inputNormal;
    NormalCloud::Ptr remNormal;
    PointCloudC colCloud;
    float minRes;
    int iterationCount;
    float initDist;
    float mergeAngle;
    float mergeDist;
    int minPoints;
    Eigen::Vector3f minValues;
    Eigen::Vector3f maxValues;

    bool mergeFlag;
    bool updateFlag;
    int updateInterval;

    double overallNeighTime;

    /** \brief Constructor for clusterAnalysis. */
    splitMerge ();

    inline void getNeighTime(double& value)
    {
    	value = overallNeighTime;
    }

    /** \brief Destructor for clusterAnalysis. */
//    virtual
//    ~splitMerge ()
//    {
//    }

    inline void setUpdateFlag(bool value)
    {
        updateFlag=value;
    }
    inline void setMergeFlag(bool value)
    {
        mergeFlag=value;
    }
    inline void setMinInlier(int value)
    {
        minPoints=value;
    }
    inline void setUpdateInterval(int value)
    {
        updateInterval = value;
    }

    inline void setInitDist(double value)
    {
        initDist = value;
    }
    inline void setMergeMaxAngle(double value)
    {
        mergeAngle = value;
    }
    inline void setMergeMaxDist(double value)
    {
        mergeDist = value;
    }

    inline void setInputCloud(PointCloudT::Ptr input){
        inputCloud=input;
    }

    inline void setNormalCloud(NormalCloud::Ptr normals){
        inputNormal=normals;
    }

    inline void setIterationCount(int iterCount)
    {
        iterationCount=iterCount;
    }

    void splitProcess();
    void mergeProcess(regions& output);
    void getNeighbour(int regIdx, std::set<int>& neighbours);

    PointCloudC getColorCloud();
    PointCloudC getMergedCloud();
private:


};
