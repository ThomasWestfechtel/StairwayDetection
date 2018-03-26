#ifndef VOXSAC
#define VOXSAC


#include <boost/multi_array.hpp>
#include <cassert>

#include <pcl/point_types.h>
#include <stairs/segmentPatch.h>
#include <pcl/common/time.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/filters/extract_indices.h>

#include <stairs/regions.h>



typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::Normal Normal;
typedef pcl::PointXYZRGB PointTC;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointTC> PointCloudC;

typedef boost::shared_ptr <std::vector<int> > IndicesPtr;

typedef typename pcl::octree::OctreePointCloud<pcl::PointXYZ> OctreeT;
typedef typename OctreeT::LeafContainer LeafContainer;


class voxSAC {
public:
    float minVoxelSize;
    float voxelSize;
    int decGrowSize;
    float minGrowSize;

    bool pointConsistency;

    Eigen::Vector4f centerPoint;

    int iterAmount;
    Eigen::Vector3d minBound;
    Eigen::Vector3d maxBound;
    Eigen::Vector3i uppLimit;
    std::vector<std::pair<float,int> > planeSeeds;
    std::vector<bool> involvedWithOther;
    std::vector<bool> indiceDeleted;
    std::vector<pcl::PointIndices> globalPoints;
    std::vector<pcl::PointIndices> initGlobalPoints;
    IndicesPtr usedIndices;
    regions priComp;
    std::vector<segmentPatch> segList;
    std::vector<segmentPatch> initSegList;

    float initDensity;
    float maxPlaneInitDist;
    bool angularInitCompFlag;
    float sampleDensity;

    float planeSacAngle;
    float planeSacDist;
    bool angularGrowCompFlag;
    float planeSacGrow;

    int minPlaneSize;

    std::map<std::vector<int>,int> occupMap;
    std::map<std::vector<int>,int>::iterator occupMapIter;

    bool planeUpdate;
    bool planeSacPointUpdate;
    int planeUpdateInterval;

    int currRun;
    float maxInitAng;
    int origCloudSize;
    float densAtOneMeter;
    float measureHorAng;
    Eigen::Vector2f measureVerAng;

    double overallNeighTime;

    PointCloudC colCloud;
    PointCloudT inputCloud;
    NormalCloud inputNormals;

    voxSAC();

    inline void setCloudSize(int input)
    {
        origCloudSize = input;
    }
    inline void setHorizontalMeasurementAngle(float input)
    {
    	measureHorAng = input/180*M_PI;
    }
    inline void setVerticalMeasurementAngle(Eigen::Vector2f input)
    {
    	measureVerAng = input/180*M_PI;
    }

    inline void getNeighTime (double& output)
    {
    	output = overallNeighTime;
    }

    inline void setPointIDConsistency(bool value)
    {
    	pointConsistency = value;
    }


    inline void setPlaneSacUpdate(bool value)
    {
        planeUpdate = value;
    }
    inline void setPlaneSacPointUpdate (bool value)
    {
        planeSacPointUpdate = value;
    }
    inline void setPlaneSacUpdateInterval(int value)
    {
        planeUpdateInterval = value;
    }
    inline void setMinPlaneSize (int value)
    {
        minPlaneSize=value;
    }


    inline void setPlaneSacAngle (float value)
    {
        planeSacAngle = cos(value/180*M_PI);
    }
    inline void setPlaneSacDist (float value)
    {
        planeSacDist = value;
    }
    inline void setPlaneSacGrowFactor (float value)
    {
    	planeSacGrow = value;
    }
    inline void setPlaneSacCompFlag(bool value)
    {
    	angularGrowCompFlag = value;
    }


    inline void setPlaneInitMaxDist(float value)
    {
        maxPlaneInitDist=value;
    }
    inline void setPlaneInitMaxAng(float value)
    {
    	maxInitAng = cos(value/180.0*M_PI);
    }
    inline void setAngularInitCompFlag(bool value)
    {
    	angularInitCompFlag = value;
    }
    inline void setPlaneInitDensity(float value)
    {
    	std::cout<<value<<std::endl;
        initDensity=value;
    }
    inline void setSampleDensity(float value)
    {
        sampleDensity = value;
    }


    inline void setIterations(int value)
    {
        iterAmount=value;
    }
    inline void setMinVoxSize(float value)
    {
        minVoxelSize = value;
    }
    inline void setDecreaseGrowFactor (int value)
    {
    	decGrowSize = value;
    }
    inline void setMinGrowSize(float size)
    {
    	minGrowSize=size;
    }


    inline void setInputCloud(PointCloudT::Ptr input)
    {
        inputCloud=*input;
    }
    inline void setNormalCloud(NormalCloud::Ptr normal)
    {
        inputNormals=*normal;
    }



    void run(regions& output);

    void analyse();
    void merging();
    void addNeighbors(std::vector<int>& liste, int segIdx, Eigen::Vector4f planeCoeff);
    bool checkOverlap(Eigen::Vector3f cubePos, Eigen::Vector4f planeCoeff);
    void updatePlaneCoeff(Eigen::Vector4f& planeCoeff,Eigen::Vector4f& centPoint,PointCloudT tempCloud);

private:
};

inline bool
comparePlane (std::pair<float, int> i, std::pair<float, int> j)
{
    return (i.first < j.first);
}

#endif // voxSAC
