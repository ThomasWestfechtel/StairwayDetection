/*
 * segmentPatch.hpp
 *
 *  Created on: Jan 30, 2015
 *      Author: tom
 */

#include <stairs/segmentPatch.h>


void segmentPatch::writeSegment(std::ofstream& writeFile)
{
	writeFile<<segmentCloud.size()<<" Points"<<"\n";
	for(size_t points = 0; points < segmentCloud.size(); points++)
	{
		writeFile<<segmentCloud.at(points).x<<" "<<segmentCloud.at(points).y<<" "<<segmentCloud.at(points).z <<"\n";
	}
}

void segmentPatch::createBorder()
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    transform(0,0)=eigen_vectors(0,2);
    transform(0,1)=eigen_vectors(1,2);
    transform(0,2)=eigen_vectors(2,2);

    transform(1,0)=eigen_vectors(0,1);
    transform(1,1)=eigen_vectors(1,1);
    transform(1,2)=eigen_vectors(2,1);

    transform(2,0)=eigen_vectors(0,0);
    transform(2,1)=eigen_vectors(1,0);
    transform(2,2)=eigen_vectors(2,0);

    transform.col(3) = -(transform*segmentCentroid);

    transform(3,3)=1;

    pcl::PointCloud<pcl::PointXYZ> tempCloud;

    pcl::transformPointCloud (segmentCloud,tempCloud, transform);

    std::map<int,Eigen::Vector2f> d1Map;
    std::map<int,Eigen::Vector2f> d2Map;
    std::map<int,Eigen::Vector2i> d1Pos;
    std::map<int,Eigen::Vector2i> d2Pos;

	std::map<int,Eigen::Vector2f>::iterator mapIter;
	std::map<int,Eigen::Vector2i>::iterator mapPosIter;

    for(size_t pointIdx = 0; pointIdx < tempCloud.size(); pointIdx++)
    {
    	float xPoint = tempCloud.at(pointIdx).x;
    	float yPoint = tempCloud.at(pointIdx).y;

    	int d1 = round(xPoint/0.05);
    	int d2 = round(yPoint/0.05);

    	bool foundD1 = false;
    	bool foundD2 = false;

        mapIter = d1Map.find(d2);
        mapPosIter = d1Pos.find(d2);

        if(mapIter == d1Map.end() )
        {
        	Eigen::Vector2f tempVec;
        	tempVec << xPoint, xPoint;
        	Eigen::Vector2i tempPos;
        	tempPos << pointIdx, pointIdx;
        	d1Map.insert(std::make_pair(d2,tempVec));
        	d1Pos.insert(std::make_pair(d2,tempPos));
        }
        else
        {
        	if(xPoint < mapIter->second[0])
        	{
        		mapIter->second[0] = xPoint;
        		mapPosIter->second[0] = pointIdx;
        	}
        	if(xPoint > mapIter->second[1])
        	{
        		mapIter->second[1] = xPoint;
        		mapPosIter->second[1] = pointIdx;
        	}
        }

        mapIter = d2Map.find(d1);
        mapPosIter = d2Pos.find(d1);

        if(mapIter == d2Map.end() )
        {
        	Eigen::Vector2f tempVec;
        	tempVec << yPoint, yPoint;
        	Eigen::Vector2i tempPos;
        	tempPos << pointIdx, pointIdx;
        	d2Map.insert(std::make_pair(d1,tempVec));
        	d2Pos.insert(std::make_pair(d1,tempPos));
        }
        else
        {
        	if(yPoint < mapIter->second[0])
        	{
        		mapIter->second[0] = yPoint;
        		mapPosIter->second[0] = pointIdx;
        	}
        	if(yPoint > mapIter->second[1])
        	{
        		mapIter->second[1] = yPoint;
        		mapPosIter->second[1] = pointIdx;
        	}
        }
	}
    for(mapPosIter = d1Pos.begin(); mapPosIter != d1Pos.end(); mapPosIter++)
    {
    	borderPoints.push_back(segmentCloud.at(mapPosIter->second[0]));
    	borderPoints.push_back(segmentCloud.at(mapPosIter->second[1]));
    }
    for(mapPosIter = d2Pos.begin(); mapPosIter != d2Pos.end(); mapPosIter++)
    {
    	borderPoints.push_back(segmentCloud.at(mapPosIter->second[0]));
    	borderPoints.push_back(segmentCloud.at(mapPosIter->second[1]));
    }
}

void segmentPatch::readSegment(std::ifstream& loadFile)
{
	std::string linestream;
	std::stringstream lines;
	int pointSize;
	getline(loadFile,linestream);
	lines.str("");
	lines.clear();
	lines << linestream;
	lines >> pointSize;

	if(pointSize < 3)
	{
		std::cout<<linestream<<std::endl;
		std::cout<< lines.str()<<std::endl;
		std::cout<<"There are "<<pointSize<<" points"<<std::endl;
	}

	for(size_t points = 0; points < pointSize; points++)
	{
		getline(loadFile,linestream);
		lines.str("");
		lines.clear();
		lines << linestream;
		PointT readPoint;
		lines >> readPoint.x >> readPoint.y >> readPoint.z;
		segmentCloud.push_back(readPoint);
		segmentIndices->indices.push_back(points);
	}
	if(pointSize != segmentCloud.size())
	{
		std::cout<<"Size mismatch!"<<std::endl;
		std::cout<<pointSize<<" and " <<segmentCloud.size()<<" "<<linestream<<std::endl;
	}
}
// Have to adjust as vectors are not orthogonal //
void segmentPatch::getRailLength(Eigen::Vector3f stairDir, Eigen::Vector3f refPoint, Eigen::Vector2f& result)
{
    std::map<int,Eigen::Vector2f> movingMap;
    std::map<int,Eigen::Vector2f>::iterator mapIter;

    float heightFactor = stairDir[2];
    stairDir[2]=0;
    heightFactor = heightFactor / stairDir.norm();
    stairDir.normalize();

    Eigen::Vector3f distanceDirection;
    distanceDirection << 0,0,1;

    for(int pointIdx = 0; pointIdx < segmentCloud.size(); pointIdx++)
    {
        Eigen::Vector3f relativePoint;
        relativePoint[0] = segmentCloud.at(pointIdx).x;
        relativePoint[1] = segmentCloud.at(pointIdx).y;
        relativePoint[2] = segmentCloud.at(pointIdx).z;

        relativePoint-=refPoint;

        float dist = relativePoint.dot(stairDir);

        relativePoint[2] += dist * heightFactor;
        int vecPosition = floor((relativePoint[2])/0.1);

        mapIter = movingMap.find(vecPosition);

        if(mapIter == movingMap.end() )
        {
        	Eigen::Vector2f tempVec;
        	tempVec << dist, dist;
        	movingMap.insert(std::make_pair(vecPosition,tempVec));
        }
        else
        {
        	if(dist < mapIter->second[0])
        		mapIter->second[0] = dist;
        	if(dist > mapIter->second[1])
        		mapIter->second[1] = dist;
        }
    }

    std::vector<float> minExtensionVector(movingMap.size());
    std::vector<float> maxExtensionVector(movingMap.size());
    int mapToVecCounter = 0;
    for(mapIter = movingMap.begin(); mapIter != movingMap.end(); mapIter++)
    {
    	minExtensionVector[mapToVecCounter] = mapIter->second[0];
    	maxExtensionVector[mapToVecCounter] = mapIter->second[1];
    	mapToVecCounter++;
    }
    sort(minExtensionVector.begin(), minExtensionVector.end());
	sort(maxExtensionVector.begin(), maxExtensionVector.end());
	result[0] = minExtensionVector.at(floor(minExtensionVector.size()*0));
	result[1] = maxExtensionVector.at(floor(maxExtensionVector.size()*1)-1);
}

// No thoughts yet //
void segmentPatch::getRailWidth(Eigen::Vector3f stairDir, Eigen::Vector3f refPoint, Eigen::Vector2f& result)
{
    std::map<int,Eigen::Vector2f> movingMap;
    std::map<int,Eigen::Vector2f>::iterator mapIter;

    Eigen::Vector3f zDirVec;
    zDirVec << 0,0,1;
    stairDir[2] = 0;
    stairDir.normalize();
    Eigen::Vector3f widthDirVec;
    widthDirVec = stairDir.cross(zDirVec);
    widthDirVec.normalize();

    for(int pointIdx = 0; pointIdx < segmentCloud.size(); pointIdx++)
    {
        Eigen::Vector3f relativePoint;
        relativePoint[0] = segmentCloud.at(pointIdx).x;
        relativePoint[1] = segmentCloud.at(pointIdx).y;
        relativePoint[2] = segmentCloud.at(pointIdx).z;

        relativePoint-= refPoint;

        int vecPosition = floor(relativePoint[2]/0.1);

        float dist = relativePoint.dot(widthDirVec);

        mapIter = movingMap.find(vecPosition);

        if(mapIter == movingMap.end() )
        {
        	Eigen::Vector2f tempVec;
        	tempVec << dist, dist;
        	movingMap.insert(std::make_pair(vecPosition,tempVec));
        }
        else
        {
        	if(dist < mapIter->second[0])
        		mapIter->second[0] = dist;
        	if(dist > mapIter->second[1])
        		mapIter->second[1] = dist;
        }
    }

    std::vector<float> minExtensionVector(movingMap.size());
    std::vector<float> maxExtensionVector(movingMap.size());
    int mapToVecCounter = 0;
    for(mapIter = movingMap.begin(); mapIter != movingMap.end(); mapIter++)
    {
    	minExtensionVector[mapToVecCounter] = mapIter->second[0];
    	maxExtensionVector[mapToVecCounter] = mapIter->second[1];
    	mapToVecCounter++;
    }
    sort(minExtensionVector.begin(), minExtensionVector.end());
	sort(maxExtensionVector.begin(), maxExtensionVector.end());
	result[0] = minExtensionVector.at(floor(minExtensionVector.size()*0));
	result[1] = maxExtensionVector.at(floor(maxExtensionVector.size()*1)-1);
}

// Have to adjust for the slope of the stairs //
void segmentPatch::getRailHeight(Eigen::Vector3f stairDir, Eigen::Vector3f refPoint, Eigen::Vector2f& result)
{
    std::map<int,Eigen::Vector2f> movingMap;
    std::map<int,Eigen::Vector2f>::iterator mapIter;

    float heightFactor = stairDir[2];
    stairDir[2]=0;
    heightFactor = heightFactor / stairDir.norm();
    stairDir.normalize();

    Eigen::Vector3f distanceDirection;
    distanceDirection << 0,0,1;

    for(int pointIdx = 0; pointIdx < segmentCloud.size(); pointIdx++)
    {
        Eigen::Vector3f relativePoint;
        relativePoint[0] = segmentCloud.at(pointIdx).x;
        relativePoint[1] = segmentCloud.at(pointIdx).y;
        relativePoint[2] = segmentCloud.at(pointIdx).z;

        relativePoint-=refPoint.head(3);

        float heightMultiplier = relativePoint.dot(stairDir);
        relativePoint[2] -= heightMultiplier * heightFactor;

        int vecPosition = floor((relativePoint.dot(stairDir))/0.1);

        float dist = relativePoint.dot(distanceDirection);

        mapIter = movingMap.find(vecPosition);

        if(mapIter == movingMap.end() )
        {
        	Eigen::Vector2f tempVec;
        	tempVec << dist, dist;
        	movingMap.insert(std::make_pair(vecPosition,tempVec));
        }
        else
        {
        	if(dist < mapIter->second[0])
        		mapIter->second[0] = dist;
        	if(dist > mapIter->second[1])
        		mapIter->second[1] = dist;
        }
    }

    std::vector<float> minExtensionVector(movingMap.size());
    std::vector<float> maxExtensionVector(movingMap.size());
    int mapToVecCounter = 0;
    for(mapIter = movingMap.begin(); mapIter != movingMap.end(); mapIter++)
    {
    	minExtensionVector[mapToVecCounter] = mapIter->second[0];
    	maxExtensionVector[mapToVecCounter] = mapIter->second[1];
    	mapToVecCounter++;
    }
    sort(minExtensionVector.begin(), minExtensionVector.end());
	sort(maxExtensionVector.begin(), maxExtensionVector.end());
	result[0] = minExtensionVector.at(floor(minExtensionVector.size()*0));
	result[1] = maxExtensionVector.at(floor(maxExtensionVector.size()*1)-1);
}

void segmentPatch::getHeight()
{
    std::map<int,Eigen::Vector2f> heightMap;
    std::map<int,Eigen::Vector2f>::iterator mapIter;

    Eigen::Vector3f eigVec;
    eigVec << 0,0,1;
    Eigen::Vector3f secVec;
    Eigen::Vector3f normTemp = segmentCoefficient.head(3);
    secVec = eigVec.cross(normTemp);
    secVec.normalize();

    for(int pointIdx = 0; pointIdx < segmentCloud.size(); pointIdx++)
    {
        Eigen::Vector3f relativePoint;
        relativePoint[0] = segmentCloud.at(pointIdx).x;
        relativePoint[1] = segmentCloud.at(pointIdx).y;
        relativePoint[2] = segmentCloud.at(pointIdx).z;

        relativePoint-=segmentCentroid.head(3);

        int vecPosition = floor((relativePoint.dot(secVec))/0.1);

        float dist = relativePoint.dot(eigVec);

        mapIter = heightMap.find(vecPosition);

        if(mapIter == heightMap.end() )
        {
        	Eigen::Vector2f tempVec;
        	tempVec << dist, dist;
        	heightMap.insert(std::make_pair(vecPosition,tempVec));
        }
        else
        {
        	if(dist < mapIter->second[0])
        		mapIter->second[0] = dist;
        	if(dist > mapIter->second[1])
        		mapIter->second[1] = dist;
        }
    }

    std::vector<float> minHeightVectorTest(heightMap.size());
    std::vector<float> maxHeightVectorTest(heightMap.size());
    int mapToVecCounter = 0;
    for(mapIter = heightMap.begin(); mapIter != heightMap.end(); mapIter++)
    {
    	minHeightVectorTest[mapToVecCounter] = mapIter->second[0];
    	maxHeightVectorTest[mapToVecCounter] = mapIter->second[1];
    	mapToVecCounter++;
    }
    sort(minHeightVectorTest.begin(), minHeightVectorTest.end());
	sort(maxHeightVectorTest.begin(), maxHeightVectorTest.end());
	height[0] = minHeightVectorTest.at(floor(minHeightVectorTest.size()*0));
	height[1] = maxHeightVectorTest.at(floor(maxHeightVectorTest.size()*1)-1);
}

void segmentPatch::getTreadDepth(Eigen::Vector3f stairDir)
{

	std::map<int,Eigen::Vector2f> depthMap;
	std::map<int,Eigen::Vector2f>::iterator mapIter;

	Eigen::Vector3f secVec;
	Eigen::Vector3f normTemp; // = segmentCoefficient.head(3);
	normTemp << 0,0,1;
	secVec = stairDir.cross(normTemp);
	secVec.normalize();

	for(int pointIdx = 0; pointIdx < segmentCloud.size(); pointIdx++)
	{
		Eigen::Vector3f relativePoint;

		relativePoint[0] = segmentCloud.at(pointIdx).x;
		relativePoint[1] = segmentCloud.at(pointIdx).y;
		relativePoint[2] = segmentCloud.at(pointIdx).z;

		relativePoint-=segmentCentroid.head(3);

		int vecPosition = floor((relativePoint.dot(secVec))/0.1);

		float dist = relativePoint.dot(stairDir);

        mapIter = depthMap.find(vecPosition);

        if(mapIter == depthMap.end() )
        {
        	Eigen::Vector2f tempVec;
        	tempVec << dist, dist;
        	depthMap.insert(std::make_pair(vecPosition,tempVec));
        }
        else
        {
        	if(dist < mapIter->second[0])
        		mapIter->second[0] = dist;
        	if(dist > mapIter->second[1])
        		mapIter->second[1] = dist;
        }
	}

    std::vector<float> minDepthVector(depthMap.size());
    std::vector<float> maxDepthVector(depthMap.size());
    std::vector<float> diffDepthVector(depthMap.size());
    int mapToVecCounter = 0;
    for(mapIter = depthMap.begin(); mapIter != depthMap.end(); mapIter++)
    {
    	minDepthVector[mapToVecCounter] = mapIter->second[0];
    	maxDepthVector[mapToVecCounter] = mapIter->second[1];
    	diffDepthVector[mapToVecCounter] = maxDepthVector[mapToVecCounter] - minDepthVector[mapToVecCounter];
    	mapToVecCounter++;
    }

	sort(maxDepthVector.begin(), maxDepthVector.end());
	sort(minDepthVector.begin(), minDepthVector.end());
	depth[0] = minDepthVector.at(floor(minDepthVector.size()*0));
	depth[1] = maxDepthVector.at(floor(maxDepthVector.size()*1)-1);

	sort(diffDepthVector.begin(), diffDepthVector.end());
	depthVariance[0]=diffDepthVector.at(floor(maxDepthVector.size()*1)-1);
	depthVariance[1]=0;
//	for(size_t vecIdx = 0; vecIdx < maxDepthVector.size(); vecIdx++)
//	{
//		depthVariance[0] += pow((minDepthVector.at(vecIdx)-depth[0]),2);
//		depthVariance[1] += pow((maxDepthVector.at(vecIdx)-depth[1]),2);
//	}
//	depthVariance /= maxDepthVector.size();
}

// Get the absolute Width //
void segmentPatch::getWidth(Eigen::Vector3f pos, Eigen::Vector3f dirVec)
{
	std::vector<float> widthVec;

	widthVec.reserve(segmentCloud.size());

    for(int pointIdx = 0; pointIdx < segmentCloud.size(); pointIdx++)
    {
        Eigen::Vector3f relativePoint;
        relativePoint[0] = segmentCloud.at(pointIdx).x;
        relativePoint[1] = segmentCloud.at(pointIdx).y;
        relativePoint[2] = segmentCloud.at(pointIdx).z;

        relativePoint= relativePoint - pos;

        float dist = relativePoint.dot(dirVec);
        widthVec.push_back(dist);
    }
    sort(widthVec.begin(),widthVec.end());

	width[0]=widthVec[0];
	width[1]=widthVec[widthVec.size() - 1];

//	Eigen::Vector4f tempCentroid;
//	Eigen::Vector2f tempDepth;
//	Eigen::Vector2f tempDepthVar;
//	tempCentroid = segmentCentroid;
//	tempDepth = depth;
//	tempDepthVar = depthVariance;
//	segmentCentroid.head(3) = pos;
//
//	getDepth(dirVec);
//
//	width = depth;
//	segmentCentroid = tempCentroid;
//	depth = tempDepth;
//	depthVariance = tempDepthVar;
}

// Get the mean height //
void segmentPatch::getMaxHeight()
{
    //std::cout<<"MAX Mean Height"<<std::endl;
    Eigen::Vector3f dirVector;
    dirVector[0] = eigen_vectors(0,2);
    dirVector[1] = eigen_vectors(1,2);
    dirVector[2] = eigen_vectors(2,2);

    std::map<int,float> maxHeightMap;
	std::map<int,float>::iterator mapIter;

    for(int pointIdx = 0; pointIdx < segmentCloud.size(); pointIdx++)
    {
        Eigen::Vector3f relativePoint;
        relativePoint[0] = segmentCloud.at(pointIdx).x;
        relativePoint[1] = segmentCloud.at(pointIdx).y;
        relativePoint[2] = segmentCloud.at(pointIdx).z;

        int vecPosition = floor(((relativePoint-segmentCentroid.head(3)).dot(dirVector))/0.1);
        float dist = relativePoint[2];

        mapIter = maxHeightMap.find(vecPosition);

        if(mapIter == maxHeightMap.end() )
        {
        	maxHeightMap.insert(std::make_pair(vecPosition,dist));
        }
        else
        {
        	if(dist > mapIter->second)
        		mapIter->second = dist;
        }
    }

    std::vector<float> maxHeightVector(maxHeightMap.size());
    int mapToVecCounter = 0;
    for(mapIter = maxHeightMap.begin(); mapIter != maxHeightMap.end(); mapIter++)
    {
    	maxHeightVector[mapToVecCounter] = mapIter->second;
    	mapToVecCounter++;
    }

    sort(maxHeightVector.begin(), maxHeightVector.end());

    maxHeight = maxHeightVector.at(floor(maxHeightVector.size()*1)-1);
}

void segmentPatch::initializeExtensions()
{
    //std::cout<<"Get the mean dimensions"<<std::endl;

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    transform(0,0)=eigen_vectors(0,2);
    transform(0,1)=eigen_vectors(1,2);
    transform(0,2)=eigen_vectors(2,2);

    transform(1,0)=eigen_vectors(0,1);
    transform(1,1)=eigen_vectors(1,1);
    transform(1,2)=eigen_vectors(2,1);

    transform(2,0)=eigen_vectors(0,0);
    transform(2,1)=eigen_vectors(1,0);
    transform(2,2)=eigen_vectors(2,0);

    transform.col(3) = -(transform*segmentCentroid);

    transform(3,3)=1;

    pcl::PointCloud<pcl::PointXYZ> tempCloud;

    pcl::transformPointCloud (segmentCloud,tempCloud, transform);

    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    pcl::getMinMax3D(tempCloud,min_pt,max_pt);
    dimensions=(max_pt-min_pt).head(3);

    std::map<int,Eigen::Vector2f> d1Map;
    std::map<int,Eigen::Vector2f> d2Map;
    std::map<int,Eigen::Vector2f> d3Map;

	std::map<int,Eigen::Vector2f>::iterator mapIter;

    for(size_t pointIdx = 0; pointIdx < tempCloud.size(); pointIdx++)
    {
    	float xPoint = tempCloud.at(pointIdx).x;
    	float yPoint = tempCloud.at(pointIdx).y;
    	float zPoint = tempCloud.at(pointIdx).z;
    	int d1 = round(xPoint/0.1);
    	int d2 = round(yPoint/0.1);

    	bool foundD1 = false;
    	bool foundD2 = false;

        mapIter = d1Map.find(d2);

        if(mapIter == d1Map.end() )
        {
        	Eigen::Vector2f tempVec;
        	tempVec << xPoint, xPoint;
        	d1Map.insert(std::make_pair(d2,tempVec));
        }
        else
        {
        	if(xPoint < mapIter->second[0])
        		mapIter->second[0] = xPoint;
        	if(xPoint > mapIter->second[1])
        		mapIter->second[1] = xPoint;
        }

        mapIter = d2Map.find(d1);

        if(mapIter == d2Map.end() )
        {
        	Eigen::Vector2f tempVec;
        	tempVec << yPoint, yPoint;
        	d2Map.insert(std::make_pair(d1,tempVec));
        }
        else
        {
        	if(yPoint < mapIter->second[0])
        		mapIter->second[0] = yPoint;
        	if(yPoint > mapIter->second[1])
        		mapIter->second[1] = yPoint;
        }

        mapIter = d3Map.find(d1);

        if(mapIter == d3Map.end() )
        {
        	Eigen::Vector2f tempVec;
        	tempVec << zPoint, zPoint;
        	d3Map.insert(std::make_pair(d1,tempVec));
        }
        else
        {
        	if(zPoint < mapIter->second[0])
        		mapIter->second[0] = zPoint;
        	if(zPoint > mapIter->second[1])
        		mapIter->second[1] = zPoint;
        }
    }

	std::vector<float> d1DimensionsMin (d1Map.size());
	std::vector<float> d1DimensionsMax (d1Map.size());

	std::vector<float> d2DimensionsMin (d2Map.size());
	std::vector<float> d2DimensionsMax (d2Map.size());

	std::vector<float> d3DimensionsMin (d3Map.size());
	std::vector<float> d3DimensionsMax (d3Map.size());

    int mapToVecCounter = 0;
    for(mapIter = d1Map.begin(); mapIter != d1Map.end(); mapIter++)
    {
    	d1DimensionsMin[mapToVecCounter] = mapIter->second[0];
    	d1DimensionsMax[mapToVecCounter] = mapIter->second[1];
    	mapToVecCounter++;
    }
    mapToVecCounter = 0;
    for(mapIter = d2Map.begin(); mapIter != d2Map.end(); mapIter++)
    {
    	d2DimensionsMin[mapToVecCounter] = mapIter->second[0];
    	d2DimensionsMax[mapToVecCounter] = mapIter->second[1];
    	mapToVecCounter++;
    }
    mapToVecCounter = 0;
    for(mapIter = d3Map.begin(); mapIter != d3Map.end(); mapIter++)
    {
    	d3DimensionsMin[mapToVecCounter] = mapIter->second[0];
    	d3DimensionsMax[mapToVecCounter] = mapIter->second[1];
    	mapToVecCounter++;
    }

    sort(d1DimensionsMin.begin(), d1DimensionsMin.end());
    sort(d1DimensionsMax.begin(), d1DimensionsMax.end());
    sort(d2DimensionsMin.begin(), d2DimensionsMin.end());
    sort(d2DimensionsMax.begin(), d2DimensionsMax.end());
    sort(d3DimensionsMin.begin(), d3DimensionsMin.end());
    sort(d3DimensionsMax.begin(), d3DimensionsMax.end());

    middleExtensions[0] = (d1DimensionsMax.at(floor(d1DimensionsMax.size()*1)-1))-(d1DimensionsMin.at(floor(d1DimensionsMin.size()*0)));
    middleExtensions[1] = (d2DimensionsMax.at(floor(d2DimensionsMax.size()*1)-1))-(d2DimensionsMin.at(floor(d2DimensionsMin.size()*0)));
    middleExtensions[2] = (d3DimensionsMax.at(floor(d3DimensionsMax.size()*1)-1))-(d3DimensionsMin.at(floor(d3DimensionsMin.size()*0)));


//    std::cout<<middleExtensions - dimensions<<std::endl;

}

void segmentPatch::analyse(PointCloudT::Ptr wholeCloud)
{
    pcl::computeMeanAndCovarianceMatrix (*wholeCloud, globalIndices, segmentCovariance, segmentCentroid);
    pcl::eigen33(segmentCovariance, eigen_vectors, eigen_values);

    segmentCoefficient[0] = eigen_vectors(0,0);
    segmentCoefficient[1] = eigen_vectors(1,0);
    segmentCoefficient[2] = eigen_vectors(2,0);
    segmentCoefficient[3] = 0;
    segmentCoefficient[3] = -1 * segmentCoefficient.dot (segmentCentroid);

    Eigen::Vector4f vp = Eigen::Vector4f::Zero ();
    vp -= segmentCentroid;
    float cos_theta = vp.dot (segmentCoefficient);
    if (cos_theta < 0)
    {
    	segmentCoefficient *= -1;
    	segmentCoefficient[3] = 0;
    	segmentCoefficient[3] = -1 * segmentCoefficient.dot (segmentCentroid);
    }

    // Compute the curvature surface change
    float eig_sum = segmentCovariance.coeff (0) + segmentCovariance.coeff (4) + segmentCovariance.coeff (8);
    if (eig_sum != 0)
		curvature = fabsf (eigen_values[0] / eig_sum);
    else
		curvature = 0;
}


void segmentPatch::analyse()
{
    if(segmentCloud.size() != segmentIndices->indices.size())
    {
//    	std::cout<<"This should not be called "<<std::endl;
        segmentIndices->indices.clear();
        for(int indicesAdder=0; indicesAdder<segmentCloud.size(); indicesAdder++)
            segmentIndices->indices.push_back(indicesAdder);
    }
    pcl::computeMeanAndCovarianceMatrix (segmentCloud, segmentIndices->indices, segmentCovariance, segmentCentroid);
    pcl::eigen33(segmentCovariance, eigen_vectors, eigen_values);

    segmentCoefficient[0] = eigen_vectors(0,0);
    segmentCoefficient[1] = eigen_vectors(1,0);
    segmentCoefficient[2] = eigen_vectors(2,0);
    segmentCoefficient[3] = 0;
    segmentCoefficient[3] = -1 * segmentCoefficient.dot (segmentCentroid);

    Eigen::Vector4f vp = Eigen::Vector4f::Zero ();
    vp -= segmentCentroid;
    float cos_theta = vp.dot (segmentCoefficient);
    if (cos_theta < 0)
    {
    	segmentCoefficient *= -1;
    	segmentCoefficient[3] = 0;
    	segmentCoefficient[3] = -1 * segmentCoefficient.dot (segmentCentroid);
    }

    // Compute the curvature surface change
    float eig_sum = segmentCovariance.coeff (0) + segmentCovariance.coeff (4) + segmentCovariance.coeff (8);
    if (eig_sum != 0)
		curvature = fabsf (eigen_values[0] / eig_sum);
    else
		curvature = 0;

    //initializeExtensions();
}
