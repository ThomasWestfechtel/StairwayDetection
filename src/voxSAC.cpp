#include <stairs/voxSAC.h>

voxSAC::voxSAC() :
	usedIndices(new std::vector<int> ())
{
	float horAngleLRF;
	horAngleLRF = 360;
	Eigen::Vector2f verAngleLRF;
	verAngleLRF[0] = 48;
	verAngleLRF[1] = 150;

	measureHorAng = horAngleLRF/180*M_PI;;
	measureVerAng = verAngleLRF/180*M_PI;;

	iterAmount=4;
	minVoxelSize =0.04;
	decGrowSize = 0;
	minGrowSize=0.02;

	maxPlaneInitDist = 0.02;
	maxInitAng = cos(45.0/180*M_PI);
	initDensity = 0.5;
	angularInitCompFlag = true;
	sampleDensity = 0.01;


	planeSacAngle = cos(20.0/180*M_PI);
	planeSacDist = 0.02;
	planeSacGrow = 0.05;
	angularGrowCompFlag = true;


	planeUpdate=true;
	planeSacPointUpdate=false;
	planeUpdateInterval=30;

	minPlaneSize=30;


	pointConsistency=false;
	overallNeighTime=0;
	voxelSize=0.02;
	currRun=0;


	minBound<<-10.24,-10.24,-10.24;
	maxBound<< 10.24, 10.24, 10.24;
	uppLimit<<0,0,0;
}

void voxSAC::analyse()
{
	int parentFactor = pow(2,decGrowSize);

	if(not(currRun<iterAmount-decGrowSize))
	{
		parentFactor = pow(2,iterAmount-currRun-1);
	}

	double anal_start = pcl::getTime();
    for(size_t segments = 0; segments < initSegList.size(); segments++)
    {
        if(initSegList.at(segments).segmentCloud.size() > 3)
        {
//            pcl::PointIndices segmentIndices;
//
//            for(size_t pointIdx = 0; pointIdx < initSegList.at(segments).segmentCloud.size(); pointIdx++)
//            {
//                segmentIndices.indices.push_back(pointIdx);
//            }
//
//            *initSegList.at(segments).segmentIndices=segmentIndices;

            initSegList.at(segments).analyse();
			float densityToDistance = pow(initSegList.at(segments).segmentCentroid.head(3).norm(),2);

			float angularInitDensity;

            if(angularInitCompFlag)
            {
            	Eigen::Vector3f tempLaserDir;
            	tempLaserDir = initSegList.at(segments).segmentCentroid.head(3);
            	tempLaserDir[0] -= 0.2;
            	tempLaserDir[2] -= 0.34;
            	tempLaserDir.normalize();
            	angularInitDensity = std::max(0.05,double(fabs(tempLaserDir.dot(initSegList.at(segments).segmentCoefficient.head(3)))));
            }
            else
            	angularInitDensity = 1;

            float initPointThresh =  densAtOneMeter * angularInitDensity * pow((parentFactor*voxelSize),2) / densityToDistance;


            if(initPointThresh > pow((parentFactor*voxelSize/ sampleDensity),2))
            {
            		initPointThresh = pow((parentFactor*voxelSize/ sampleDensity),2) ;
            }

			if(initSegList.at(segments).segmentCloud.size()> initDensity* initPointThresh)
			{

				initSegList.at(segments).initializeExtensions();

				if(initSegList.at(segments).dimensions[2]<maxPlaneInitDist)
				{
					std::pair<float,int> tempPlaneSeed;
					tempPlaneSeed.first = initSegList.at(segments).dimensions[2];
					tempPlaneSeed.second = segments;
					planeSeeds.push_back(tempPlaneSeed);
				}
			}

        }
    }
    double anal_end = pcl::getTime();
    std::sort (planeSeeds.begin (), planeSeeds.end (), comparePlane);
}


void voxSAC::updatePlaneCoeff(Eigen::Vector4f& planeCoeff,Eigen::Vector4f& centPoint,PointCloudT tempCloud)
{
    segmentPatch tempPatch;
    pcl::PointIndices segmentIndices;
    tempPatch.segmentCloud = tempCloud;
    for(size_t pointIdx = 0; pointIdx < tempCloud.size(); pointIdx++)
    {
        tempPatch.segmentIndices->indices.push_back(pointIdx);
    }
    tempPatch.analyse();
    planeCoeff = tempPatch.segmentCoefficient;
    centPoint = tempPatch.segmentCentroid;
}


void voxSAC::merging ()
{
    double updateTime = 0;
    double neighTime = 0;
    double sac_start = pcl::getTime();
    for (int planeSeedIdx = 0; planeSeedIdx < planeSeeds.size(); planeSeedIdx++)
    {
    	int parentFactor = pow(2,decGrowSize);

    	if(not(currRun<iterAmount-decGrowSize))
    	{
    		parentFactor = pow(2,iterAmount-currRun-1);
    	}

        int segmentIdx = planeSeeds.at(planeSeedIdx).second;

        Eigen::Vector3i parPos = initSegList.at(segmentIdx).roomPos;

        parPos *= parentFactor;

        std::vector<int> tempIds;
        std::vector<int> usedTempIdx;

		std::vector<int> segs;
		PointCloudT tempCloud;
		std::vector<int> globVec;
		segmentPatch tempSeg;
//		initSegList.at(segmentIdx).analyse();
		Eigen::Vector4f planeCoeff = initSegList.at(segmentIdx).segmentCoefficient;
		centerPoint =  initSegList.at(segmentIdx).segmentCentroid;


		for(int x1 = 0; x1 < parentFactor; x1++)
		{
			for(int y1 = 0; y1 < parentFactor; y1++)
			{
				for(int z1 = 0; z1 < parentFactor; z1++)
				{
					std::vector<int> posVector;
					posVector.push_back(parPos[0]+x1);
					posVector.push_back(parPos[1]+y1);
					posVector.push_back(parPos[2]+z1);

					occupMapIter = occupMap.find(posVector);
					if(occupMapIter != occupMap.end())
					{
						segmentIdx = occupMapIter->second;

						if(involvedWithOther[segmentIdx] == false)
						{
							if(segList.at(segmentIdx).segmentCloud.size()>0)
							{
				            	segList.at(segmentIdx).analyse();
								tempCloud+=segList.at(segmentIdx).segmentCloud;
								std::copy(segList.at(segmentIdx).globalIndices.begin(),segList.at(segmentIdx).globalIndices.end(),std::back_inserter(globVec));
								for(int pIdx = 0; pIdx < segList.at(segmentIdx).segmentCloud.size(); pIdx++)
								{
									if(indiceDeleted.at(globalPoints.at(segmentIdx).indices.at(pIdx))==false)
									{
										tempIds.push_back(globalPoints.at(segmentIdx).indices.at(pIdx));
									}
								}
								segs.push_back(segmentIdx);
								usedTempIdx.push_back(segmentIdx);

							}
						}
					}
				}
			}
		}
		int begSize = segs.size();
		for(int segI = 0; segI < begSize; segI++)
		{
			addNeighbors(segs, segs[segI], planeCoeff);
		}

		if(segs.size() > begSize -1)
		{
			for(size_t listIdx = begSize; listIdx < segs.size(); listIdx++)
			{
				Eigen::Vector3i currRoomPos = segList.at(segs.at(listIdx)).roomPos;
				std::vector<int> addedPointIdx;
				PointCloudT tempCloud2;
				std::vector<int> globVec2;
				int pointAdded = 0;
				for(size_t pointIdx = 0; pointIdx < segList.at(segs.at(listIdx)).segmentCloud.size(); pointIdx++)
				{
					if(indiceDeleted.at(globalPoints.at(segs.at(listIdx)).indices.at(pointIdx))==false)
					{
						Eigen::Vector3f currPoint;
						currPoint[0] = segList.at(segs.at(listIdx)).segmentCloud.at(pointIdx).x;
						currPoint[1] = segList.at(segs.at(listIdx)).segmentCloud.at(pointIdx).y;
						currPoint[2] = segList.at(segs.at(listIdx)).segmentCloud.at(pointIdx).z;

						Eigen::Vector3f currNormal;
						currNormal[0] = segList.at(segs.at(listIdx)).normalCloud.at(pointIdx).normal_x;
						currNormal[1] = segList.at(segs.at(listIdx)).normalCloud.at(pointIdx).normal_y;
						currNormal[2] = segList.at(segs.at(listIdx)).normalCloud.at(pointIdx).normal_z;

						if(fabs((centerPoint.head(3)-currPoint).dot(planeCoeff.head(3))) < planeSacDist)
						{
							if(fabs(currNormal.dot(planeCoeff.head(3))) > planeSacAngle)
							{
								addedPointIdx.push_back(pointIdx);
								tempCloud2.push_back(segList.at(segs.at(listIdx)).segmentCloud.at(pointIdx));
								globVec2.push_back(segList.at(segs.at(listIdx)).globalIndices[pointIdx]);
								pointAdded ++;
								if(planeUpdate && planeSacPointUpdate)
								{
									centerPoint.head(3) = (centerPoint.head(3)*(tempCloud.size() + pointAdded -1) + currPoint) / (tempCloud.size() + pointAdded);
									if(planeCoeff.head(3).dot(currNormal) > 0)
										planeCoeff.head(3) = (tempCloud.size() + pointAdded -1) * planeCoeff.head(3) + currNormal;
									else
										planeCoeff.head(3) = (tempCloud.size() + pointAdded -1) * planeCoeff.head(3) - currNormal;
									planeCoeff.head(3).normalize();
								}
							}
						}
					}
				}

				if(pointAdded > 0)
				{
					if(planeUpdate && not(planeSacPointUpdate))
					{
						if(pointAdded > planeUpdateInterval)
						{
							double up_start = pcl::getTime();
							Eigen::Vector4f updateCoeff;
							Eigen::Vector4f updatePoint;
							updatePlaneCoeff(updateCoeff, updatePoint,tempCloud2);
							if(planeCoeff.head(3).dot(updateCoeff.head(3)) > 0)
								planeCoeff.head(3) = planeCoeff.head(3) * tempCloud.size() + updateCoeff.head(3) * tempCloud2.size();
							else
								planeCoeff.head(3) = planeCoeff.head(3) * tempCloud.size() - updateCoeff.head(3) * tempCloud2.size();
							planeCoeff.head(3).normalize();

							centerPoint = (centerPoint * tempCloud.size() + updatePoint * tempCloud2.size())/(tempCloud.size() + tempCloud2.size());
							double up_end = pcl::getTime();
							updateTime += up_end-up_start;
						}
					}
					double ng_start = pcl::getTime();




					float densityToDistance = pow(segList.at(segs.at(listIdx)).segmentCentroid.head(3).norm(),2);

					float angularGrowDensity;

		            if(angularGrowCompFlag)
		            {
		            	Eigen::Vector3f tempLaserDir;
		            	tempLaserDir = segList.at(segs.at(listIdx)).segmentCentroid.head(3) - centerPoint.head(3);
		            	tempLaserDir = segList.at(segs.at(listIdx)).segmentCentroid.head(3) - tempLaserDir.dot(planeCoeff.head(3)) * planeCoeff.head(3);
		            	tempLaserDir[0] -= 0.2;
		            	tempLaserDir[2] -= 0.34;
		            	tempLaserDir.normalize();
		            	angularGrowDensity = std::max(0.05,double(fabs(tempLaserDir.dot(planeCoeff.head(3)))));
		            }
		            else
		            	angularGrowDensity = 1;

		            float initPointThresh =  densAtOneMeter * angularGrowDensity * pow(voxelSize,2) / densityToDistance;


		            if(initPointThresh > pow((voxelSize/ sampleDensity),2))
		            {
		            		initPointThresh = pow((voxelSize/ sampleDensity),2) ;
		            }

//		            float factor = planeSacGrow* pow((voxelSize/sampleDensity),2)/densityToDistance * angularGrowDensity;


		            if(pointAdded > planeSacGrow* initPointThresh)
		            {
		            	addNeighbors(segs, segs.at(listIdx), planeCoeff);
		            }

					double ng_end = pcl::getTime();
					neighTime+=ng_end-ng_start;

					for(int pIdx = 0; pIdx < addedPointIdx.size(); pIdx++)
					{

						tempIds.push_back(globalPoints.at(segs.at(listIdx)).indices.at(addedPointIdx.at(pIdx)));

					}
					tempCloud+=tempCloud2;
					globVec.insert(globVec.end(), globVec2.begin(), globVec2.end());
					usedTempIdx.push_back(segs.at(listIdx));
				}
			}
		}

		if(tempCloud.size()>minPlaneSize)
		{
			tempSeg.segmentCloud=tempCloud;
			tempSeg.globalIndices=globVec;
			tempSeg.segmentCoefficient = planeCoeff;
			priComp.push_back(tempSeg);
			for(int tID = 0; tID < tempIds.size(); tID ++)
			{
				usedIndices->push_back(tempIds[tID]);
				indiceDeleted.at(tempIds[tID])=true;
			}
			for(int aID = 0; aID < usedTempIdx.size(); aID++)
				involvedWithOther[usedTempIdx[aID]]=true;
		}
    }
    double sac_end = pcl::getTime();
    overallNeighTime = neighTime;
}

bool voxSAC::checkOverlap(Eigen::Vector3f cubePos, Eigen::Vector4f planeCoeff)
{
	if(planeSacDist <= 2*voxelSize)
		return true;
    Eigen::Vector3f cornerPos;
    float tolerance = planeSacDist;
    cornerPos[0] = minBound[0] + cubePos[0] * voxelSize - tolerance;
    cornerPos[1] = minBound[1] + cubePos[1] * voxelSize - tolerance;
    cornerPos[2] = minBound[2] + cubePos[2] * voxelSize - tolerance;

    bool belowPlane = false;
    bool abovePlane = false;

    for(int xCorn = 0; xCorn < 2; xCorn++)
    {
        for(int yCorn=0;yCorn < 2; yCorn++)
        {
            for(int zCorn=0;zCorn<2;zCorn++)
            {
                Eigen::Vector3f testPos;
                testPos[0] = cornerPos[0] + xCorn * (voxelSize+2*tolerance);
                testPos[1] = cornerPos[1] + yCorn * (voxelSize+2*tolerance);
                testPos[2] = cornerPos[2] + zCorn * (voxelSize+2*tolerance);
                if((testPos-centerPoint.head(3)).dot(planeCoeff.head(3)) > 0)
                    belowPlane = true;
                else abovePlane =true;

                if(abovePlane && belowPlane)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

void voxSAC::addNeighbors(std::vector<int>& liste, int segIdx, Eigen::Vector4f planeCoeff)
{
    int expandFactor = 1;
    Eigen::Vector3i occP = segList.at(segIdx).roomPos;
    for(int xIdx = -1*expandFactor; xIdx <= 1*expandFactor; xIdx++)
    {
        for(int yIdx = -1*expandFactor; yIdx <= 1*expandFactor; yIdx++)
        {
            for(int zIdx = -1*expandFactor; zIdx <= 1*expandFactor; zIdx++)
            {
            	if(occP[0]+xIdx > 0 && occP[1]+yIdx > 0 && occP[2]+zIdx > 0)
            	{
					std::vector<int> posVector;
					posVector.push_back(occP[0]+xIdx);
					posVector.push_back(occP[1]+yIdx);
					posVector.push_back(occP[2]+zIdx);

					occupMapIter = occupMap.find(posVector);
					if(occupMapIter != occupMap.end())
					{

						Eigen::Vector3f cubePos;
						cubePos << occP[0]+xIdx, occP[1]+yIdx, occP[2]+zIdx;
						if(checkOverlap(cubePos, planeCoeff))
						{
							int toAdd = occupMapIter->second;
							std::vector<int>::iterator vecIter;
							vecIter = find(liste.begin(),liste.end(),toAdd);
							if(vecIter == liste.end())
							{
								liste.push_back(toAdd);
							}
						}
					}
            	}
            }
        }
    }
}


void voxSAC::run(regions& output)
{
	origCloudSize = inputCloud.size();

	densAtOneMeter = origCloudSize / (measureHorAng * (cos(measureVerAng[0])- cos(measureVerAng[1])));

	std::deque<int> globalIdxMap;
	globalIdxMap.resize(inputCloud.size());
    for(int assignIdx = 0; assignIdx < inputCloud.size(); assignIdx++)
    {
        globalIdxMap[assignIdx]=assignIdx;
    }

    voxelSize=pow(2,iterAmount-decGrowSize-1)*minVoxelSize;
    int parentFactor = pow(2,decGrowSize);

    int segIterator = 0;

    for(int iteration=0;iteration<iterAmount;iteration++)
    {
    	currRun = iteration;
    	if(iteration == 0 && voxelSize < minGrowSize)
    	{
    		std::cout<<"Growing size is larger than the Voxel Size..."<<std::endl;
    		return;
    	}
    	if(voxelSize < minGrowSize)
    	{
    		voxelSize *= 2;
    		parentFactor/=2;
    	}
    	segIterator = 0;

    	occupMap.clear();
        segList.clear();
        initSegList.clear();
        globalPoints.clear();
        usedIndices->clear();
        indiceDeleted.clear();
        involvedWithOther.clear();
        for(int assignIdx = 0; assignIdx < inputCloud.size(); assignIdx++)
            indiceDeleted.push_back(false);

        pcl::octree::OctreePointCloud<pcl::PointXYZ> otSAC (voxelSize);
        otSAC.setInputCloud (inputCloud.makeShared());
        otSAC.addPointsFromInputCloud ();
        otSAC.getBoundingBox(minBound[0],minBound[1],minBound[2],maxBound[0],maxBound[1],maxBound[2]);



        segmentPatch initSeg;

        bool firstRun = true;
        std::vector<int> parVecPos;

        pcl::octree::OctreePointCloud<pcl::PointXYZ>::LeafNodeIterator itOtSAC (&otSAC);
        do
        {
        	segmentPatch tempSeg;
        	pcl::PointIndices globalPoint;

            std::vector<int> indicesVec;
            LeafContainer lf;
            lf=itOtSAC.getLeafContainer();

            lf.getPointIndices(indicesVec);

            int leafSize = indicesVec.size();
            tempSeg.roomPos[0]=itOtSAC.getCurrentOctreeKey().x;
            tempSeg.roomPos[1]=itOtSAC.getCurrentOctreeKey().y;
            tempSeg.roomPos[2]=itOtSAC.getCurrentOctreeKey().z;

            std::vector<int> curParVecPos;
            curParVecPos.push_back(tempSeg.roomPos[0]/parentFactor);
            curParVecPos.push_back(tempSeg.roomPos[1]/parentFactor);
            curParVecPos.push_back(tempSeg.roomPos[2]/parentFactor);
        	if(firstRun)
        	{
        		firstRun = false;
        		parVecPos = curParVecPos;
                initSeg.roomPos[0] = parVecPos[0];
                initSeg.roomPos[1] = parVecPos[1];
                initSeg.roomPos[2] = parVecPos[2];
        	}
        	else if(curParVecPos != parVecPos)
        	{
                initSegList.push_back(initSeg);
                initSeg.globalIndices.clear();
                initSeg.segmentCloud.clear();
                initSeg.normalCloud.clear();
                parVecPos = curParVecPos;
                initSeg.roomPos[0] = parVecPos[0];
                initSeg.roomPos[1] = parVecPos[1];
                initSeg.roomPos[2] = parVecPos[2];
        	}

            tempSeg.segmentCentroid[0] = (tempSeg.roomPos[0]+0.5) * voxelSize +  minBound[0];
            tempSeg.segmentCentroid[1] = (tempSeg.roomPos[1]+0.5) * voxelSize +  minBound[1];
            tempSeg.segmentCentroid[2] = (tempSeg.roomPos[2]+0.5) * voxelSize +  minBound[2];

            Eigen::Vector3i rPos = tempSeg.roomPos;

            std::vector<int> posVector;
            posVector.push_back(rPos[0]);
            posVector.push_back(rPos[1]);
            posVector.push_back(rPos[2]);

            occupMap.insert(make_pair(posVector,segIterator));

            segIterator++;

            for(size_t voxelPoint=0;voxelPoint<leafSize;voxelPoint++)
            {
                globalPoint.indices.push_back(indicesVec[voxelPoint]);
                tempSeg.globalIndices.push_back(globalIdxMap[indicesVec[voxelPoint]]);
                initSeg.globalIndices.push_back(globalIdxMap[indicesVec[voxelPoint]]);
                tempSeg.segmentCloud.push_back(inputCloud.at(indicesVec[voxelPoint]));
                initSeg.segmentCloud.push_back(inputCloud.at(indicesVec[voxelPoint]));
                tempSeg.normalCloud.push_back(inputNormals.at(indicesVec[voxelPoint]));
                initSeg.normalCloud.push_back(inputNormals.at(indicesVec[voxelPoint]));
            }
            globalPoints.push_back(globalPoint);
            segList.push_back(tempSeg);
            involvedWithOther.push_back(false);

        }while(*++itOtSAC);

        analyse();

        merging();

        if(pointConsistency)
        {
        	std::list<int> tempList;
        	std::copy(globalIdxMap.begin(),globalIdxMap.end(),back_inserter(tempList));
			std::sort(usedIndices->begin(),usedIndices->end());
			int lastDel = 0;
			std::list<int>::iterator it1;
			it1 = tempList.begin();
			for(int delID = 0; delID < usedIndices->size(); delID++)
			{
				advance (it1,usedIndices->at(delID)-lastDel);
				lastDel=usedIndices->at(delID)+1;
				it1=tempList.erase(it1);
			}

			globalIdxMap.clear();
			std::copy(tempList.begin(),tempList.end(),back_inserter(globalIdxMap));
        }

        pcl::ExtractIndices<PointT> eifilter (true); // Initializing with true will allow us to extract the removed indices
        eifilter.setInputCloud (inputCloud.makeShared());
        eifilter.setIndices (usedIndices);
        eifilter.setNegative (true);
        eifilter.filter (inputCloud);

        pcl::ExtractIndices<Normal> eifilterNorm (true); // Initializing with true will allow us to extract the removed indices
        eifilterNorm.setInputCloud (inputNormals.makeShared());
        eifilterNorm.setIndices (usedIndices);
        eifilterNorm.setNegative (true);
        eifilterNorm.filter (inputNormals);

        voxelSize/=2;

    }
    for(int segLabel = 0; segLabel < priComp.size(); segLabel++)
        priComp.at(segLabel).segmentLabel=segLabel;
    priComp.analyse();
    priComp.getExtensions();
    output.regs.reserve(priComp.size());
    output=priComp;
}


