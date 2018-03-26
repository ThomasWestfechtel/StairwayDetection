// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <stairs/preanalysis.h>
#include <stairs/regions.h>
#include <stairs/regiongrowing.h>
#include <stairs/voxSAC.h>
#include <stairs/splitmerge.h>
#include <stairs/planeshape.h>
#include <stairs/recognition.h>
#include <stairs/StairVector.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::Normal Normal;
typedef pcl::PointXYZRGB PointTC;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointTC> PointCloudC;


int main (int argc, char *argv[])
{  
	if(argc < 3)
	{
		std::cout<<"Not enough arguments - State input point cloud and result point cloud file"<<std::endl;
	}

// Loading input point cloud //

	int return_status;

	std::cout<<"Starting loading point cloud"<<std::endl;
	double loadS = pcl::getTime();

	PointCloudT::Ptr mainCloud;
	mainCloud.reset (new PointCloudT);

	return_status = pcl::io::loadPCDFile (argv[1], *mainCloud);
	if (return_status != 0)
	{
		PCL_ERROR("Error reading point cloud %s\n", argv[1]);
		return -1;
	}
	double loadE = pcl::getTime();
	std::cout<<"Loading took: "<<loadE-loadS<<std::endl;

// Starting preanalysis //

	std::cout<<"Starting preanalysis"<<std::endl;
    double preAS = pcl::getTime();

    Preanalysis pre;
    NormalCloud::Ptr prepNomalCloud;
    prepNomalCloud.reset(new NormalCloud);
    PointCloudT floorPC;
    PointCloudC prepNorMap;

    pre.run(mainCloud, prepNomalCloud, prepNorMap, floorPC);
    double preAE = pcl::getTime();
    std::cout<<"Preanalysis took: "<<preAE-preAS<<std::endl;



// Starting segmentation //
    std::cout<<"Starting segmentation"<<std::endl;
    int mode = 0;
    double segS = pcl::getTime();
    regions segRegions;
    if(mode == 0)
    {
        std::cout<<"Using Region Growing algorihtm"<<std::endl;
        RegionGrowing reGrow;
        reGrow.setInputCloud(mainCloud);
        reGrow.setNormalCloud(prepNomalCloud);
        reGrow.run(segRegions);
    }
    if(mode == 1)
    {
        std::cout<<"Using Voxel SAC algorihtm"<<std::endl;
        voxSAC voxelSAC;
        voxelSAC.setInputCloud(mainCloud);
        voxelSAC.setNormalCloud(prepNomalCloud);
        voxelSAC.run(segRegions);
    }
    if(mode ==2)
    {
        std::cout<<"Using Split & Merge algorihtm"<<std::endl;
        splitMerge sam;
        sam.setInputCloud(mainCloud);
        sam.setNormalCloud(prepNomalCloud);
        sam.splitProcess();
        sam.mergeProcess(segRegions);
    }
    double segE = pcl::getTime();
    std::cout<<"Segmentation took: "<<segE-segS<<std::endl;

// Starting plane finder - plane extraction //

    std::cout<<"Starting plane finder"<<std::endl;

    double pfS = pcl::getTime();
    planeshape psProc;
    regions stairTreads;
    regions stairRisers;
    psProc.setInputRegions(segRegions);
    psProc.filterSc(stairTreads, stairRisers);

    double pfE = pcl::getTime();
    std::cout<<"Plane filter took: "<<pfE-pfS<<std::endl;

// Starting graph-based stair detection //
    std::cout<<"Starting graph-based detection"<<std::endl;
    StairVector detectedStairs;

    double refS = pcl::getTime();
    recognition stairDetect;
    stairDetect.setInputRegions(segRegions);
    stairDetect.setStairTreadRegions(stairTreads);
    stairDetect.setStairRiseRegions(stairRisers);
    stairDetect.run(detectedStairs);
    double refE = pcl::getTime();

    std::cout<<"There are treads: "<<stairTreads.size()<<std::endl;
    std::cout<<"There are risers: "<<stairRisers.size()<<std::endl;

    std::cout<<"Refinement took: "<<refE-refS<<std::endl;
    std::cout<<"Total time  took: "<<refE-loadS<<std::endl;

// Printing out the results //

    bool colorByPart = true;

    PointCloudC resultCloud;

    bool addBackGround = true;
    if(addBackGround)
    {
    	for(size_t pointID = 0; pointID < mainCloud->size(); pointID ++)
    	{
    		PointTC backPoint;
    		backPoint.x = mainCloud->points[pointID].x;
    		backPoint.y = mainCloud->points[pointID].y;
    		backPoint.z = mainCloud->points[pointID].z;
    		backPoint.r=255;
    		backPoint.g=255;
    		backPoint.b=255;
    		resultCloud.push_back(backPoint);
    	}
    }

    std::cout<<"Detected stairways: "<<detectedStairs.size()<<std::endl;
    if(detectedStairs.size()>0)
    {
		for(int stairCoeffIdx =0; stairCoeffIdx < detectedStairs.size(); stairCoeffIdx++)
		{
			Stairs stairCoefficients;
			stairCoefficients = detectedStairs.at(stairCoeffIdx);

			float steigung = atan(stairCoefficients.dir[2] / sqrt(pow(stairCoefficients.dir[0],2) + pow(stairCoefficients.dir[1],2)));

			std::cout<<std::endl<<"Step depth:   "<<round(1000*sqrt(pow(stairCoefficients.dir[0],2) + pow(stairCoefficients.dir[1],2)))<<std::endl;
			std::cout<<"Step height:  "<<round(1000*stairCoefficients.dir[2])<<std::endl;
			std::cout<<"Step width:   "<<round(1000*stairCoefficients.width)<<std::endl;
			std::cout<<"Slope is:     "<<round(100*steigung/M_PI*180)<<std::endl;
			std::cout<<"Amount of stair parts: "<<stairCoefficients.size()<<std::endl<<std::endl;

			float stairAngle = atan2(stairCoefficients.dir[1],stairCoefficients.dir[0]);
			float xStairDist = stairCoefficients.pos[0];
			float yStairDist = stairCoefficients.pos[1];

			Eigen::Vector2f sepDist;
			sepDist[0] = cos(stairAngle) * xStairDist + sin(stairAngle) * yStairDist;
			sepDist[1] = - sin(stairAngle) * xStairDist + cos(stairAngle) * yStairDist;

	        std::cout<<"Dist in X is: "<<round(1000*(stairCoefficients.pos[0]))<<std::endl;
	        std::cout<<"Dist in Y is: "<<round(1000*stairCoefficients.pos[1])<<std::endl;

	        std::cout<<"Dist par is:  "<<round(1000*sepDist[0])<<std::endl;
	        std::cout<<"Dist ort is:  "<<round(1000*sepDist[1])<<std::endl;
			std::cout<<"Anchor point is: "<<stairCoefficients.anchPoint<<std::endl;

			std::cout<<"Angle is:     "<<round(100*atan2(stairCoefficients.dir[1],stairCoefficients.dir[0])/M_PI*180)<<std::endl;

			if(colorByPart)
				resultCloud += detectedStairs.getColoredParts(stairCoeffIdx);
			else
				resultCloud += detectedStairs.getColoredCloud(stairCoeffIdx);
		}
    }

// Saving output point cloud //
    if(detectedStairs.size()>0)
    {
        std::cout<<"Saving result pointcloud"<<std::endl;
        pcl::io::savePCDFile(argv[2],resultCloud);
    }
}

