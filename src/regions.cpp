/*
 * regions.hpp
 *
 *  Created on: Feb 2, 2015
 *      Author: tom
 */

#include <stairs/regions.h>


//using namespace std;

void regions::sortByRefs ()
{
    std::sort (regs.begin (), regs.end (), compareRefs);
}

void regions::createBorders()
{
	for(int regID = 0; regID < regs.size(); regID++)
	{
		regs[regID].createBorder();
	}
}

void regions::showBorders(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	int linesAdded = 0;
	PointCloudC colored_cloud;
  if (!regs.empty ())
  {
//	  std::cout<<"There are "<<regs.size()<<" regions"<<std::endl;
    srand (static_cast<unsigned int> (time (0)));
    std::vector<unsigned char> colors;
    for (size_t i_segment = 0; i_segment < regs.size (); i_segment++)
    {
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
    }

    int next_color = 0;
    std::stringstream ssf ("Line");
    for (size_t i_segment = 0; i_segment < regs.size (); i_segment++)
    {
//    	std::cout<<"Start next"<<std::endl;

    	PointCloudT linesCloud;

    	linesCloud = regs.at(i_segment).borderPoints;

    	std::stringstream ssfn;
    	ssfn << ssf.str() << i_segment << "seg";

    	int counter = 0;
    	while(linesCloud.size() > 3 && counter < 4)
    	{
			PointCloudT resultCloud;
			pcl::SampleConsensusModelLine<PointT>::Ptr model_l (new pcl::SampleConsensusModelLine<PointT>(linesCloud.makeShared()));
			pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_l);
			ransac.setDistanceThreshold (.05);
			ransac.computeModel();
			std::vector<int> inliers;
			ransac.getInliers(inliers);
			Eigen::VectorXf model_coefficients;
			ransac.getModelCoefficients(model_coefficients);
//			std::cout<<"COPYING"<<std::endl;
			pcl::copyPointCloud<pcl::PointXYZ>(linesCloud, inliers, resultCloud);

            pcl::ExtractIndices<PointT> eifilter (true); // Initializing with true will allow us to extract the removed indices
            eifilter.setInputCloud (linesCloud.makeShared());

            boost::shared_ptr <std::vector<int> > remIndices (new std::vector<int>);

            for(int indIdx = 0; indIdx < inliers.size(); indIdx++)
            	remIndices->push_back(inliers[indIdx]);

//            std::cout<<"Should reduce: "<<inliers.size()<<std::endl;
//            std::cout<<"Before filter: "<<linesCloud->size()<<std::endl;
            eifilter.setIndices (remIndices);
            eifilter.setNegative (true);
            eifilter.filter (linesCloud);
//            std::cout<<"After filter: "<<linesCloud->size()<<std::endl;
            counter++;
//			std::cout<<"Add result"<<std::endl;
            if(resultCloud.size() > 3)
            {
            	Eigen::Vector2f minMax;
            	for(int pointIdx = 0; pointIdx < resultCloud.size(); pointIdx ++)
            	{
            		Eigen::Vector3f currPoint;
            		currPoint[0] = resultCloud.at(pointIdx).x;
            		currPoint[1] = resultCloud.at(pointIdx).y;
            		currPoint[2] = resultCloud.at(pointIdx).z;
            		float dist = (currPoint-model_coefficients.head(3)).dot(model_coefficients.segment(3,3));
            		if(pointIdx == 0)
            		{
            			minMax[0] = dist;
            			minMax[1] = dist;
            		}
            		else
            		{
            			if(dist < minMax[0])
            				minMax[0] = dist;
            			if(dist > minMax[1])
            				minMax[1] = dist;
            		}
            	}

            	PointT startPoint;
            	PointT endPoint;

            	startPoint.x = model_coefficients[0] + minMax[0] * model_coefficients[3];
            	startPoint.y = model_coefficients[1] + minMax[0] * model_coefficients[4];
            	startPoint.z = model_coefficients[2] + minMax[0] * model_coefficients[5];

            	endPoint.x = model_coefficients[0] + minMax[1] * model_coefficients[3];
            	endPoint.y = model_coefficients[1] + minMax[1] * model_coefficients[4];
            	endPoint.z = model_coefficients[2] + minMax[1] * model_coefficients[5];

				std::stringstream ssline1;
				ssline1<<ssfn.str();
				ssline1<<counter;
				double red = colors[3 * next_color];
				double green = colors[3 * next_color + 1];
				double blue = colors[3 * next_color + 2];
				viewer->addLine(startPoint,endPoint,red/255,green/255,blue/255,ssline1.str());
				linesAdded++;

				for (size_t i_point = 0; i_point < resultCloud.size(); i_point++)
				{
					pcl::PointXYZRGB point;
					point.x=resultCloud.at(i_point).x;
					point.y=resultCloud.at(i_point).y;
					point.z=resultCloud.at(i_point).z;
					point.r = colors[3 * next_color];
					point.g = colors[3 * next_color + 1];
					point.b = colors[3 * next_color + 2];
					colored_cloud.push_back(point);
				}
            }
    	}
      next_color++;
    }
  }
  viewer->removeAllPointClouds ();
  std::cout<<"There should be "<<linesAdded<<" lines"<<std::endl;
  viewer->addPointCloud(colored_cloud.makeShared(),"col_cloud");
}

void regions::saveRegion(std::string savePath)
{
	std::cout<<"Starting saving "<<regs.size()<<" regions"<<std::endl;
	double startTime = pcl::getTime();
//	generateCenterCloud();
	std::ofstream writeFile(savePath.c_str());
	writeFile<<regs.size()<<"\n";
	for(size_t cloud = 0; cloud < regs.size(); cloud++)
	{
	  writeFile<<cloud<<"Clouds: "<<"\n";
	  regs.at(cloud).writeSegment(writeFile);
	}
	std::cout<<"Adding the floor cloud with: "<<floorCloud.size()<<" Points"<<std::endl;
	writeFile<<floorCloud.size()<<" Points"<<"\n";
	for(size_t points = 0; points < floorCloud.size(); points++)
	{
		writeFile<<floorCloud.at(points).x<<" "<<floorCloud.at(points).y<<" "<<floorCloud.at(points).z <<"\n";
	}

	double finishTime = pcl::getTime();
	std::cout<<"Saving took: "<<finishTime-startTime<<"sec"<<std::endl;
}

void regions::loadRegion(std::string loadPath)
{
	regs.clear();

	std::cout<<"Started loading"<<std::endl;
	double startTime = pcl::getTime();
	std::ifstream loadFile(loadPath.c_str());
	std::string linestream;
	std::stringstream lines;
	int regionsSize;
	getline(loadFile,linestream);
	lines.str("");
	lines << linestream;
	lines >> regionsSize;
	std::cout<<"There are "<<regionsSize<<" regions"<<std::endl;

	// Read the segmented Point Clouds
	for(size_t cloud = 0; cloud < regionsSize; cloud++)
	{
		int refCloudNo;
		lines.str("");
		lines.clear();
		getline(loadFile,linestream);
		lines << linestream;
		lines >> refCloudNo;
		if(cloud != refCloudNo)
		{
			std::cout<<"NOPE: "<<cloud<<" != "<<refCloudNo<<std::endl;
			std::cout<<linestream<<std::endl;
			cloud = regionsSize;
		}
		segmentPatch testSegment;
		testSegment.readSegment(loadFile);
		testSegment.segmentLabel=cloud;
//		std::cout<<"Segment size is: "<<testSegment.segmentCloud->size()<<std::endl;
		regs.push_back(testSegment);
	}
	analyse();
	getExtensions();
	// Read floor Point Cloud
	int pointSize;
	getline(loadFile,linestream);
	lines.str("");
	lines.clear();
	lines << linestream;
	lines >> pointSize;
	for(size_t points = 0; points < pointSize; points++)
	{
		getline(loadFile,linestream);
		lines.str("");
		lines.clear();
		lines << linestream;
		PointT readPoint;
		lines >> readPoint.x >> readPoint.y >> readPoint.z;
		floorCloud.push_back(readPoint);
	}

	double finishTime = pcl::getTime();
	std::cout<<"Loading took: "<<finishTime-startTime<<"sec"<<std::endl;
}

void regions::generateCenterCloud()
{
//	std::cout<<"Center Cloud Generation"<<std::endl;
    centerCloud.clear();
    for(int regIdx = 0; regIdx < regs.size(); regIdx++)
    {
        PointT centPoint;
        centPoint.x = regs.at(regIdx).segmentCentroid[0];
        centPoint.y = regs.at(regIdx).segmentCentroid[1];
        centPoint.z = regs.at(regIdx).segmentCentroid[2];
        centerCloud.push_back(centPoint);
    }
}

PointCloudC regions::getColoredCloud ()
{
//	std::cout<<"Colored Cloud Generation"<<std::endl;

    PointCloudC colored_cloud;

  if (!regs.empty ())
  {
//	  std::cout<<"There are "<<regs.size()<<" regions"<<std::endl;
    srand (static_cast<unsigned int> (time (0)));
    std::vector<unsigned char> colors;
    for (size_t i_segment = 0; i_segment < regs.size (); i_segment++)
    {
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
    }


    int next_color = 0;
    for (size_t i_segment = 0; i_segment < regs.size (); i_segment++)
    {
      for (size_t i_point = 0; i_point < regs.at(i_segment).segmentCloud.size(); i_point++)
      {
        pcl::PointXYZRGB point;
        point.x=regs.at(i_segment).segmentCloud.at(i_point).x;
        point.y=regs.at(i_segment).segmentCloud.at(i_point).y;
        point.z=regs.at(i_segment).segmentCloud.at(i_point).z;
        point.r = colors[3 * next_color];
        point.g = colors[3 * next_color + 1];
        point.b = colors[3 * next_color + 2];
        colored_cloud.push_back(point);
      }
      next_color++;
    }
  }

  return (colored_cloud);
}


PointCloudC regions::getNormalMap ()
{
  PointCloudC colored_cloud;

  if (!regs.empty ())
  {
//      std::cout<<"There are "<<std::cout.size()<<" regions"<<std::endl;

    for (size_t i_segment = 0; i_segment < regs.size (); i_segment++)
    {
        Eigen::Vector3i colVec;
        colVec << 255,255,255;

        colVec[0] = abs(round(255*cbrt(regs.at(i_segment).segmentCoefficient[0])));
        colVec[1] = abs(round(255*cbrt(regs.at(i_segment).segmentCoefficient[1])));
        colVec[2] = abs(round(255*cbrt(regs.at(i_segment).segmentCoefficient[2])));

      for (size_t i_point = 0; i_point < regs.at(i_segment).segmentCloud.size(); i_point++)
      {
        pcl::PointXYZRGB point;
        point.x=regs.at(i_segment).segmentCloud.at(i_point).x;
        point.y=regs.at(i_segment).segmentCloud.at(i_point).y;
        point.z=regs.at(i_segment).segmentCloud.at(i_point).z;
        point.r = colVec[0];
        point.g = colVec[1];
        point.b = colVec[2];
        colored_cloud.push_back(point);
      }
    }
  }

  return (colored_cloud);
}
