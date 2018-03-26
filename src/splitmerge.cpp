#include <stairs/splitmerge.h>

splitMerge::splitMerge ()
{
	minRes = 0.04;

	iterationCount=4;
	initDist=0.04;
	mergeAngle=15;
	mergeDist=0.04;

	updateFlag=true;
	updateInterval=10;

	minPoints = 30;
	mergeFlag=true;

	uppLimit=0;

	remCloud.reset(new PointCloudT);
	remNormal.reset(new NormalCloud);
}

void splitMerge::splitProcess()
{
    remCloud = inputCloud;
    remNormal = inputNormal;

    double max_bound_x=0;
    double max_bound_y=0;
    double max_bound_z=0;
    double min_bound_x=0;
    double min_bound_y=0;
    double min_bound_z=0;
//
//    minValues << min_bound_x,min_bound_y,min_bound_z;
//    maxValues << max_bound_x,max_bound_y,max_bound_z;

    int segCounterIdx = 0;

	bool careAboutGlobal = false;
	std::deque<int> globalIdxMap;
	globalIdxMap.resize(inputCloud->size());
    for(int assignIdx = 0; assignIdx < inputCloud->size(); assignIdx++)
    {
        globalIdxMap[assignIdx]=assignIdx;
    }


    for(int iterIdx = 0; iterIdx < iterationCount; iterIdx++)
    {
        IndicesPtr remIndices (new Indices);
        float resolution = pow(2,iterationCount-iterIdx-1)*minRes;
        OctreeT octSAM(resolution);
        if(iterIdx != 0)
        	octSAM.defineBoundingBox(min_bound_x,min_bound_y,min_bound_z,max_bound_x,max_bound_y,max_bound_z);

        octSAM.setInputCloud (remCloud);
        octSAM.addPointsFromInputCloud ();

        if(iterIdx == 0)
        	octSAM.getBoundingBox(min_bound_x,min_bound_y,min_bound_z,max_bound_x,max_bound_y,max_bound_z);

        /* Looking for uniform surfaces */
        double octUS_start = pcl::getTime();
        pcl::octree::OctreePointCloud<pcl::PointXYZ>::LeafNodeIterator itOtSAC (&octSAM);

        do
        {
          int minPoints = 3;
          std::vector<int> indicesVec;
          LeafContainer lf;
          lf=itOtSAC.getLeafContainer();
          lf.getPointIndices(indicesVec);
          int leafSize = indicesVec.size();
          if(leafSize>=minPoints)
          {
              segmentPatch tempSeg;
              for(size_t voxelPoint=0;voxelPoint<leafSize;voxelPoint++)
              {
                  tempSeg.segmentCloud.push_back(remCloud->at(indicesVec.at(voxelPoint)));
                  tempSeg.normalCloud.push_back(remNormal->at(indicesVec.at(voxelPoint)));
                  tempSeg.segmentIndices->indices.push_back(voxelPoint);
                  tempSeg.globalIndices.push_back(globalIdxMap[indicesVec[voxelPoint]]);
              }

              tempSeg.analyse();

              tempSeg.initializeExtensions();

              tempSeg.voxSize = pow(2,iterationCount-iterIdx-1);
              tempSeg.segmentLabel = -1;

            if(tempSeg.dimensions[2]<initDist)
            {
                  int xPos =itOtSAC.getCurrentOctreeKey().x*(pow(2,iterationCount-iterIdx-1));
                  int yPos =itOtSAC.getCurrentOctreeKey().y*(pow(2,iterationCount-iterIdx-1));
                  int zPos =itOtSAC.getCurrentOctreeKey().z*(pow(2,iterationCount-iterIdx-1));

                  tempSeg.roomPos[0]=xPos;
                  tempSeg.roomPos[1]=yPos;
                  tempSeg.roomPos[2]=zPos;

                  for(int xTemp = 0; xTemp < pow(2,iterationCount-iterIdx-1); xTemp++)
                  {
                      for(int yTemp = 0; yTemp < pow(2,iterationCount-iterIdx-1); yTemp++)
                      {
                          for(int zTemp = 0; zTemp < pow(2,iterationCount-iterIdx-1); zTemp++)
                          {
							  std::vector<int> posVector;
							  posVector.push_back(xPos+xTemp);
							  posVector.push_back(yPos+yTemp);
							  posVector.push_back(zPos+zTemp);
                              occupMap.insert(make_pair(posVector,segCounterIdx));
                          }
                      }
                  }
                  segCounterIdx++;


                priComp.push_back(tempSeg);
//				std::cout<<indicesVec.size()<<std::endl;
                for(int indIdx = 0; indIdx < indicesVec.size(); indIdx++)
                {
                    remIndices->push_back(indicesVec.at(indIdx));
                }
            }
          }
        }while(*++itOtSAC);

//        std::cout<<remIndices->size()<<std::endl;


//        std::cout<<"Before: "<<remCloud->size()<<std::endl;
        if(remIndices->size()>0)
        {
            if(careAboutGlobal)
            {
               	std::list<int> tempList;
				std::copy(globalIdxMap.begin(),globalIdxMap.end(),back_inserter(tempList));
				std::sort(remIndices->begin(),remIndices->end());
				int lastDel = 0;
				std::list<int>::iterator it1;
				it1 = tempList.begin();
				for(int delID = 0; delID < remIndices->size(); delID++)
				{
	//				if(delID % 100 == 0)
	//					std::cout<<delID<<std::endl;
					advance (it1,remIndices->at(delID)-lastDel);
					lastDel=remIndices->at(delID)+1;
					it1=tempList.erase(it1);
				}
				globalIdxMap.clear();
				std::copy(tempList.begin(),tempList.end(),back_inserter(globalIdxMap));
            }

            double rem_start = pcl::getTime();
            pcl::ExtractIndices<PointT> eifilter (true); // Initializing with true will allow us to extract the removed indices
            eifilter.setInputCloud (remCloud);
            eifilter.setIndices (remIndices);
            eifilter.setNegative (true);
            eifilter.filter (*remCloud);

            pcl::ExtractIndices<Normal> eifilterNorm (true); // Initializing with true will allow us to extract the removed indices
            eifilterNorm.setInputCloud (remNormal);
            eifilterNorm.setIndices (remIndices);
            eifilterNorm.setNegative (true);
            eifilterNorm.filter (*remNormal);
            double rem_end = pcl::getTime();
//            std::cout<<"Removing took: "<<rem_end-rem_start<<std::endl;
        }
//        else
//            std::cout<<"Nope found"<<std::endl;

//		std::cout<<"After: "<<remCloud->size()<<std::endl;
        double octUS_end = pcl::getTime();
//		std::cout<<"Preanalysis took: "<<octUS_end-octUS_start<<std::endl;

    }

}



void splitMerge::mergeProcess(regions& output)
{
	int doubleTrouble=0;

	std::map<int,std::vector<int> > mergeMap;
	std::map<int,std::vector<int> >::iterator mergeMapIt;
	std::map<int,std::vector<int> > mergeMapBase;
	std::map<int,std::vector<int> >::iterator mergeMapBaseIt;
	double neighTime = 0.0;
    output.clear();
    if(mergeFlag)
    {
		int labelCount = 0;
		for(int regIdx = 0; regIdx < priComp.size(); regIdx++)
		{
			bool newLabel = false;
			if(priComp.at(regIdx).segmentLabel==-1)
			{
				priComp.at(regIdx).segmentLabel=labelCount;
				std::vector<int> tempVec;
				tempVec.push_back(regIdx);
				mergeMapBase[labelCount]=tempVec;
				labelCount++;
				newLabel = true;
			}

			std::set<int> neighbours;
			Eigen::Vector4f currCoeff = priComp.at(regIdx).segmentCoefficient;
			Eigen::Vector3f currCor = priComp.at(regIdx).segmentCentroid.head(3);

            double neighStart = pcl::getTime();
            getNeighbour(regIdx, neighbours);
            double neighEnd = pcl::getTime();

            neighTime += neighEnd - neighStart;

			for(std::set<int>::iterator neighIt = neighbours.begin(); neighIt != neighbours.end(); neighIt++)
			{
				int currId = *neighIt;
				Eigen::Vector4f nxtCoeff = priComp.at(currId).segmentCoefficient;
				Eigen::Vector3f nxtCor = priComp.at(currId).segmentCentroid.head(3);
				float angle = acos(fabs(currCoeff.head(3).dot(nxtCoeff.head(3))))/M_PI*180;
				float distance = fabs((currCor-nxtCor).dot(currCoeff.head(3)));
				if(angle < mergeAngle && distance < mergeDist)
				{
					if(priComp.at(currId).segmentLabel == -1)
					{
						priComp.at(currId).segmentLabel = priComp.at(regIdx).segmentLabel;
						mergeMapBase.at(priComp.at(regIdx).segmentLabel).push_back(currId);
					}
					else
					{
						mergeMapIt = mergeMap.find(priComp.at(currId).segmentLabel);
						if (mergeMapIt == mergeMap.end())
						{
						  std::vector<int> newVec;
						  newVec.push_back(priComp.at(regIdx).segmentLabel);
						  mergeMap[priComp.at(currId).segmentLabel] = newVec;
						}
						else
						{
							mergeMapIt->second.push_back(priComp.at(regIdx).segmentLabel);
						}

						mergeMapIt = mergeMap.find(priComp.at(regIdx).segmentLabel);
						if (mergeMapIt == mergeMap.end())
						{
						  std::vector<int> newVec;
						  newVec.push_back(priComp.at(currId).segmentLabel);
						  mergeMap[priComp.at(regIdx).segmentLabel] = newVec;
						}
						else
						{
						  mergeMapIt->second.push_back(priComp.at(currId).segmentLabel);
						}

					}
                }
            }
		}

		std::set<int> globIndicSet;

		std::set<int> labelsDone;
		std::set<int> voxelsDone;

		for(int labelIdCount = 0; labelIdCount < labelCount-1; labelIdCount++)
		{
			if(labelsDone.find(labelIdCount) == labelsDone.end())
			{
				std::vector<int> labelToDo;

				labelToDo.push_back(labelIdCount);
				int labelId;
				for(int ltd = 0; ltd < labelToDo.size(); ltd++)
				{
					labelId = labelToDo[ltd];
					mergeMapIt = mergeMap.find(labelId);
					if (mergeMapIt != mergeMap.end())
					{
						for(int insertId = 0; insertId < (mergeMapIt->second.size()); insertId++)
						{
							if(std::find(labelToDo.begin(), labelToDo.end(), mergeMapIt->second[insertId]) == labelToDo.end())
							{
								if(labelsDone.find(mergeMapIt->second[insertId]) == labelsDone.end())
									labelToDo.push_back(mergeMapIt->second[insertId]);
							}
						}
					}

				}

				segmentPatch tempSeg;
				int labID = 0;

				for(int ltd = 0; ltd < labelToDo.size(); ltd++)
				{
					labelId = labelToDo[ltd];
					std::vector<int> voxelsToDo;
					mergeMapIt = mergeMapBase.find(labelId);
					if (mergeMapIt != mergeMapBase.end())
						voxelsToDo = mergeMapIt->second;
					else
					{
						std::cout<<"Not good =("<<std::endl;
						std::cout<<labelId<<std::endl;
					}

					for(int vtd = 0; vtd < voxelsToDo.size(); vtd++)
					{
//						if(voxelsDone.find(voxelsToDo[vtd])==voxelsDone.end())
						for(int pid = 0; pid < priComp.at(voxelsToDo[vtd]).segmentCloud.size(); pid++)
						{
							tempSeg.segmentCloud.push_back (priComp.at(voxelsToDo[vtd]).segmentCloud[pid]);
							tempSeg.globalIndices.push_back(priComp.at(voxelsToDo[vtd]).globalIndices[pid]);

							if(globIndicSet.find(priComp.at(voxelsToDo[vtd]).globalIndices[pid]) == globIndicSet.end())
								globIndicSet.insert(priComp.at(voxelsToDo[vtd]).globalIndices[pid]);
							else
								doubleTrouble++;

							tempSeg.segmentLabel = labID;
							labID++;
						}
						voxelsDone.insert(voxelsToDo[vtd]);
					}
					labelsDone.insert(labelId);
				}
				if(tempSeg.segmentCloud.size() >= minPoints)
					output.push_back(tempSeg);
			}
		}

    }

    else
    {
//        output.regs = priComp;
    	int labelId = 0;
        for(size_t iterator = 0; iterator < priComp.size(); iterator++)
        {
            if(priComp.at(iterator).segmentCloud.size() > minPoints)
            {
            	priComp.at(iterator).segmentLabel=labelId;
                output.push_back(priComp.at(iterator));
                labelId++;
            }
        }
        std::cout<<"There should be "<<priComp.size()<<" regions"<<std::endl;
    }

    overallNeighTime = neighTime;
    output.analyse();
    output.getExtensions();
}



void splitMerge::getNeighbour(int regIdx, std::set<int>& neighbours)
{
    int expandFactor = 1;
    Eigen::Vector3i occP = priComp.at(regIdx).roomPos;
    int size = priComp.at(regIdx).voxSize;
    for(int xIdx = -1*expandFactor; xIdx < 1*expandFactor+size; xIdx++)
    {
    	int yIncrement = 1;
        for(int yIdx = -1*expandFactor; yIdx < 1*expandFactor+size; yIdx+=yIncrement)
        {
        	int zIncrement = 1;
        	if (yIdx != -1 && yIdx != size && xIdx != -1 && xIdx != size)
        		zIncrement = expandFactor+size;
            for(int zIdx = -1*expandFactor; zIdx < 1*expandFactor+size; zIdx+=zIncrement)
            {
				std::vector<int> posVector;
				posVector.push_back(occP[0]+xIdx);
				posVector.push_back(occP[1]+yIdx);
				posVector.push_back(occP[2]+zIdx);
				occupMapIter = occupMap.find(posVector);
				if(occupMapIter != occupMap.end())
				{
					if(occupMapIter->second > regIdx)
					{
                        //std::cout<<"Till here"<<std::endl;
                        int toAdd = occupMapIter->second;
                        bool alrAdded = false;

						neighbours.insert(toAdd);

					}
				}
            }
        }
    }
}

PointCloudC splitMerge::getColorCloud()
{
    PointCloudC colored_cloud;

  if (!priComp.empty ())
  {

    srand (static_cast<unsigned int> (time (0)));
    std::vector<unsigned char> colors;
    for (size_t i_segment = 0; i_segment < priComp.size (); i_segment++)
    {
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
    }


    int next_color = 0;
    for (size_t i_segment = 0; i_segment < priComp.size (); i_segment++)
    {
      for (size_t i_point = 0; i_point < priComp.at(i_segment).segmentCloud.size(); i_point++)
      {
        pcl::PointXYZRGB point;
        point.x=priComp.at(i_segment).segmentCloud.at(i_point).x;
        point.y=priComp.at(i_segment).segmentCloud.at(i_point).y;
        point.z=priComp.at(i_segment).segmentCloud.at(i_point).z;
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

PointCloudC splitMerge::getMergedCloud()
{
    PointCloudC colored_cloud;

  if (!mergeComp.empty ())
  {

    srand (static_cast<unsigned int> (time (0)));
    std::vector<unsigned char> colors;
    for (size_t i_segment = 0; i_segment < mergeComp.size (); i_segment++)
    {
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
    }


    int next_color = 0;
    for (size_t i_segment = 0; i_segment < mergeComp.size (); i_segment++)
    {
        if(mergeComp.at(i_segment).segmentCloud.size()>50)
        {
          for (size_t i_point = 0; i_point < mergeComp.at(i_segment).segmentCloud.size(); i_point++)
          {
            pcl::PointXYZRGB point;
            point.x=mergeComp.at(i_segment).segmentCloud.at(i_point).x;
            point.y=mergeComp.at(i_segment).segmentCloud.at(i_point).y;
            point.z=mergeComp.at(i_segment).segmentCloud.at(i_point).z;
            point.r = colors[3 * next_color];
            point.g = colors[3 * next_color + 1];
            point.b = colors[3 * next_color + 2];
            colored_cloud.push_back(point);
          }
          next_color++;
        }
    }
  }

  return (colored_cloud);
}
