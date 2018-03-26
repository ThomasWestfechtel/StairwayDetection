#include <stairs/recognition.h>

#define sind(x) (sin(fmod((x),360) * M_PI / 180))
#define cosd(x) (cos(fmod((x),360) * M_PI / 180))

recognition::recognition()
{
	// graphMeth false for extended search - true for simple search
	graphMeth = false;
	// use least-squared method to optimize parameters as post-processing
	optimizeFlag = true;

	// Width requirement for the stairs
	widthReqVec << 0.0,10.0;

	//Stair parts have to overlap in their width
	widthFlag = true;

	// Check the angle between stair parts
	parFlag = true;
	parAngle = cos(15.0/180*M_PI);

	// Height distances
	ndFlag = true;
	nDistance << 0.11,0.24;

	// Depth distances
	pdFlag = true;
	pDistance << 0.18,0.50;

	// true for known floor - false for unknown (floor should be at z = 0.0
	floorInformation = false;


	// Update stair coefficients during graph extension - results are suboptimal - not recommended
	updateFlag = false;

	// Extend the rail beyond the detected stairway
	stairRailFlag = false;

    // Set if you want to find stairs with known parameters
    predifinedValues = false;
    preDefDepth = 0;
	preDefHeight = 0;
	preDefWidth = 0;
}


void recognition::filter()
{
//	PointCloudT groundCloud;
//	for(int xCor = -10; xCor < 11; xCor++)
//	{
//		for(int yCor = -10; yCor < 11; yCor++)
//		{
//			PointT addPoint;
//			addPoint.x = 0.1*xCor;
//			addPoint.y = 0.1*yCor;
//			addPoint.z = 0;
//			groundCloud.push_back(addPoint);
//		}
//	}

	double checkTime = 0;
	double extendTime =0;
	double whole_start = pcl::getTime();
	memTime = 0;
	widthTime = 0;
	sortTime = 0;
	getWTime = 0;

	globalAddedLabel.clear();
	stairCount = 0;
	float minStairInc = sin(10.0/180.0*M_PI);
	float maxStairInc = sin(50.0/180.0*M_PI);

	basePart = 0;
    for(int firstSeg=0; firstSeg<stairRiseRegions.size()-1; firstSeg++)
    {
    	// Check if the current segment is already used within a solution if yes skip based on the selection //
    	if(graphMeth == true)
    	{
    		if(std::find(globalAddedLabel.begin(), globalAddedLabel.end(), stairRiseRegions.at(firstSeg).segmentLabel) != globalAddedLabel.end())
    		{
    			continue;
    		}
    	}
        for(int secondSeg=firstSeg+1; secondSeg<stairRiseRegions.size();secondSeg++)
        {
        	// Check if the current segment is already used within a solution if yes skip based on the selection //
        	if(graphMeth == true)
        	{
        		if(std::find(globalAddedLabel.begin(), globalAddedLabel.end(), stairRiseRegions.at(secondSeg).segmentLabel) != globalAddedLabel.end())
        		{
        			continue;
        		}
        	}
            segmentPatch firstPatch = stairRiseRegions.at(firstSeg);
            segmentPatch secondPatch = stairRiseRegions.at(secondSeg);

            if((firstPatch.segmentCentroid - secondPatch.segmentCentroid).norm() <1) // && firstPatch.segmentCentroid[2] < 2.5)
            {
			Eigen::Vector3f dirVec;

			// Check all 3 possible direction initializations //
			for(int dirInit = 0; dirInit < 3; dirInit++)
			{

				if(dirInit == 0)
					dirVec = firstPatch.segmentCoefficient.head(3);
				else if(dirInit ==1)
				{
					dirVec = secondPatch.segmentCoefficient.head(3);
				}
				else if(dirInit ==2)
				{
					if(firstPatch.segmentCoefficient.head(2).dot(secondPatch.segmentCoefficient.head(2)) < 0)
					{
						dirVec.head(2) = secondPatch.segmentCoefficient.head(2) - firstPatch.segmentCoefficient.head(2);
					}
					else
					{
						dirVec.head(2) = secondPatch.segmentCoefficient.head(2) + firstPatch.segmentCoefficient.head(2);
					}
				}

				dirVec[2] = 0;
				dirVec.normalize();

				if((secondPatch.segmentCentroid.head(3) - firstPatch.segmentCentroid.head(3)).dot(dirVec) < 0)
					dirVec *= -1;

				// Check the horizontal distance between the planes //
				float hDist = fabs((secondPatch.segmentCentroid - firstPatch.segmentCentroid).head(3).dot(dirVec));
				if(hDist > pDistance[0] && hDist < pDistance[1] && ((firstPatch.segmentCentroid[2] > 0.1 && secondPatch.segmentCentroid[2] > 0.1) || (not(floorInformation))))
				{
					// Check the angle difference between the planes //
//					float angleDifference = fabs(firstPatch.segmentCoefficient.head(2).dot(secondPatch.segmentCoefficient.head(2)));

					Eigen::Vector2f firstNormal = firstPatch.segmentCoefficient.head(2);
					firstNormal.normalize();
					Eigen::Vector2f secondNormal = secondPatch.segmentCoefficient.head(2);
					secondNormal.normalize();
					float angleDifference = fabs(firstNormal.dot(secondNormal));

					if(angleDifference > parAngle)
					{
						// Check vertical distance between the planes //
						firstPatch.getHeight();
						secondPatch.getHeight();
						firstPatch.getMaxHeight();
						secondPatch.getMaxHeight();

						// Check all 3 possible height initializations //
						for(int heightInitMode = 0; heightInitMode < 3; heightInitMode++)
						{
							float vDist;
							if(heightInitMode == 0)
								vDist = (secondPatch.segmentCentroid[2] + secondPatch.height[1]) - (firstPatch.segmentCentroid[2] + firstPatch.height[1]);
							else if(heightInitMode == 1)
								vDist = (secondPatch.segmentCentroid[2] + secondPatch.height[0]) - (firstPatch.segmentCentroid[2] + firstPatch.height[0]);
							else if(heightInitMode == 2)
								vDist = (secondPatch.segmentCentroid[2]) - (firstPatch.segmentCentroid[2]);

//							float overlap;
//							if(vDist < 0)
//								overlap = (firstPatch.segmentCentroid[2] + firstPatch.height[0]) - (secondPatch.segmentCentroid[2] + secondPatch.height[1]);
//							else
//								overlap = (secondPatch.segmentCentroid[2] + secondPatch.height[0]) - (firstPatch.segmentCentroid[2] + firstPatch.height[1]);

							if(fabs(vDist) > nDistance[0] && fabs(vDist) < nDistance[1])// && overlap > -0.05)
							{
								if((secondPatch.segmentCentroid.head(3) - firstPatch.segmentCentroid.head(3)).dot(dirVec) < 0)
								{
									std::cout<<"???"<<std::endl;
									dirVec *= -1;
								}
								distVec = dirVec;
								if(vDist < 0)
								{
									distVec.head(2) = -distVec.head(2)*hDist;
								}
								else
								{
									distVec.head(2) = distVec.head(2)*hDist;
								}

								distVec[2] = fabs(vDist);

								Eigen::Vector3f dirVecNorm = distVec;
								dirVecNorm.normalize();

								// Check the inlination //
								if(dirVecNorm[2] < maxStairInc && dirVecNorm[2] > minStairInc)
								{
						        	stairTreads.clear();
						        	stairRises.clear();
						        	addedLabel.clear();

									stairs.push_back(Stairs());

									addedLabel.push_back(firstPatch.segmentLabel);
									stairs.at(stairCount).stairRiseCloud+=firstPatch.segmentCloud;
									stairs.at(stairCount).stairParts.push_back(firstPatch);
									stairs.at(stairCount).stairRises.push_back(firstPatch);
									stairs.at(stairCount).stairRises.at(stairs.at(stairCount).stairRises.size()-1).segmentLabel=0;
									stairRises.push_back(firstPatch);
									stairRises.at(stairRises.size()-1).segmentLabel=0;

									startSearchPoint.head(2) = firstPatch.segmentCentroid.head(2);


									if(heightInitMode == 0)
										startSearchPoint[2] = firstPatch.segmentCentroid[2] + firstPatch.height[1];
									else if(heightInitMode == 1)
										startSearchPoint[2] = firstPatch.segmentCentroid[2] + firstPatch.height[0] + fabs(vDist);
									else if(heightInitMode == 2)
										startSearchPoint[2] = firstPatch.segmentCentroid[2] + fabs(vDist)/2;

									if(predifinedValues)
									{
										std::cout<<"Using pre defined values"<<std::endl;
										double preDefFactor = preDefDepth / distVec.head(2).norm();
										distVec.head(2)*=preDefFactor;
										distVec[2] = preDefHeight;
									}

//									while(!checkSol.wasStopped())
//									{
//										checkSol.spinOnce();
//									}

									double exStart = pcl::getTime();
									find();
									double exEnd = pcl::getTime();
									extendTime += exEnd - exStart;
									double checkStart = pcl::getTime();
									check();
									double checkEnd = pcl::getTime();
									checkTime += checkEnd - checkStart;
								}
							}
						}
					}
				}
				}
			}
        }
    }
    basePart = 1;
    for(int firstSeg=0; firstSeg<stairTreadRegions.size()-1; firstSeg++)
    {
    	if(graphMeth == true)
		{
			if(std::find(globalAddedLabel.begin(), globalAddedLabel.end(), stairTreadRegions.at(firstSeg).segmentLabel) != globalAddedLabel.end())
			{
				continue;
			}
		}
        for(int secondSeg=firstSeg+1; secondSeg<stairTreadRegions.size();secondSeg++)
        {
        	// Check if the current segment is already used within a solution if yes skip based on the selection //
        	if(graphMeth == true)
        	{
        		if(std::find(globalAddedLabel.begin(), globalAddedLabel.end(), stairTreadRegions.at(secondSeg).segmentLabel) != globalAddedLabel.end())
        		{
        			continue;
        		}
        	}
            segmentPatch firstPatch = stairTreadRegions.at(firstSeg);
            segmentPatch secondPatch = stairTreadRegions.at(secondSeg);
        	if(true || firstPatch.segmentCentroid[2] < 2.5)
        	{
            // Check vertical distance //
            float vDist = secondPatch.segmentCentroid[2] - firstPatch.segmentCentroid[2];
            if(fabs(vDist) > nDistance[0] && fabs(vDist) < nDistance[1] && ((firstPatch.segmentCentroid[2] > 0.1 && secondPatch.segmentCentroid[2] > 0.1) || (not(floorInformation))))
            {


            	// Check all 3 possible direction initializations //
            	for(int dirInit = 0; dirInit < 3; dirInit++)
            	{

            		if(fabs(firstPatch.segmentCoefficient[2]*secondPatch.segmentCoefficient[2]) > parAngle)
            		{
						if(dirInit == 0)
						{
							distVec = firstPatch.eigen_vectors.col(1);
						}
						if(dirInit==1)
						{
							distVec = secondPatch.eigen_vectors.col(1);
						}
						if(dirInit==2)
						{
							distVec = secondPatch.segmentCentroid.head(3) - firstPatch.segmentCentroid.head(3);
						}

						if(distVec.dot(secondPatch.segmentCentroid.head(3) - firstPatch.segmentCentroid.head(3))<0)
							distVec *= -1;

						distVec[2] = 0;
						distVec.normalize();

						if(vDist<0)
							distVec *= -1;

						firstPatch.getTreadDepth(distVec);
						secondPatch.getTreadDepth(distVec);

						// Check all 3 possible depth initializations //
						for(int depthInitMode = 0; depthInitMode < 3; depthInitMode++)
						{
							float hDist;
							if(depthInitMode == 0)
								hDist = fabs(distVec.head(2).dot((secondPatch.segmentCentroid.head(2) + secondPatch.depth[1]*distVec.head(2)) - (firstPatch.segmentCentroid.head(2) + firstPatch.depth[1]*distVec.head(2))));
							if(depthInitMode == 1)
								hDist = fabs(distVec.head(2).dot((secondPatch.segmentCentroid.head(2) + secondPatch.depth[0]*distVec.head(2)) - (firstPatch.segmentCentroid.head(2) + firstPatch.depth[0]*distVec.head(2))));
							if(depthInitMode == 2)
								hDist = fabs(distVec.head(2).dot(secondPatch.segmentCentroid.head(2) - firstPatch.segmentCentroid.head(2)));

							if(hDist > pDistance[0] && hDist < pDistance[1])
							{

								distVec *= hDist;
								distVec[2] = fabs(vDist);

								Eigen::Vector3f dirVecNorm = distVec;
								dirVecNorm.normalize();

								// Check the inlination //
								if(dirVecNorm[2] < maxStairInc && dirVecNorm[2] > minStairInc)
								{
				                	stairTreads.clear();
				                	stairRises.clear();
				                	addedLabel.clear();

									addedLabel.push_back(firstPatch.segmentLabel);

									stairs.push_back(Stairs());

									stairs.at(stairCount).stairParts.push_back(firstPatch);
									stairs.at(stairCount).stairTreadCloud+=firstPatch.segmentCloud;
									stairs.at(stairCount).stairTreads.push_back(firstPatch);
									stairs.at(stairCount).stairTreads.at(stairs.at(stairCount).stairTreads.size()-1).segmentLabel=0;
									stairTreads.push_back(firstPatch);
									stairTreads.at(stairTreads.size()-1).segmentLabel=0;

									if(depthInitMode == 0)
									{
											startSearchPoint.head(2) = firstPatch.segmentCentroid.head(2) + (-hDist + firstPatch.depth[1])*distVec.head(2);

									}
									if(depthInitMode == 1)
									{
										startSearchPoint.head(2) = firstPatch.segmentCentroid.head(2) + firstPatch.depth[0]*distVec.head(2);
									}
									else
									{
										startSearchPoint.head(2) = firstPatch.segmentCentroid.head(2) - (hDist/2)*distVec.head(2);
									}
									startSearchPoint[2] = firstPatch.segmentCentroid[2];

									if(predifinedValues)
									{
										double preDefFactor = preDefDepth / distVec.head(2).norm();
										distVec.head(2)*=preDefFactor;
										distVec[2] = preDefHeight;
									}

									double exStart = pcl::getTime();
									find();
									double exEnd = pcl::getTime();
									extendTime += exEnd - exStart;
									double checkStart = pcl::getTime();
									check();
									double checkEnd = pcl::getTime();
									checkTime += checkEnd - checkStart;
								}
							}
						}
                    }
            	}
            }
            }
        }
    }

    double whole_end = pcl::getTime();

    whole_time = whole_end - whole_start;

//    std::cout<<"Whole process took: "<<whole_time<<" seconds"<<std::endl;
////    std::cout<<"Memory operations took: "<<memTime<<" seconds"<<std::endl;
////    std::cout<<"Width operation took: "<<widthTime<<" seconds"<<std::endl;
////    std::cout<<"Width estimation process took: "<<getWTime<<" seconds"<<std::endl;
//    std::cout<<"Extension took: "<<extendTime<<" seconds"<<std::endl;
//    std::cout<<"Checking took: "<<checkTime<<" seconds"<<std::endl;
//    std::cout<<"Sorting process took: "<<sortTime<<" seconds"<<std::endl<<std::endl;
}


void recognition::run(StairVector& output)
{
    addedLabel.clear();

    stairs.clear();

    maxStairRiseDist = 0.05;
    maxStairRiseHDist = 0.05;
    maxStairTreadDist = 0.05;
    maxStairRiseAngle = cos(30/180*M_PI);

	filter();

//    std::cout<<"Looking for circular stairways"<<std::endl;
//	//filterCircular();
//	std::cout<<"Finished looking for circular stairways"<<std::endl;

	//170703 change to false//
	bool difSolsOnly = true;

	stairs.sort();
	std::set<int> delIDX;
	std::set<int>::iterator delIDXIter;
//	std::cout<<"Sorting finished -- total of "<<stairs.size()<<" stairs."<<std::endl;
	StairVector stairSolution;
	if(difSolsOnly && graphMeth != true)
	{
		for(int stairIdx = 0; stairIdx < stairs.size()-1; stairIdx++)
		{
			delIDXIter = delIDX.find(stairIdx);
			if(delIDXIter == delIDX.end())
			{
				finalize(stairs.at(stairIdx));

				stairSolution.push_back(stairs.at(stairIdx));
				std::set<int> usedPlanes;
				for(int treadIdx = 0; treadIdx < stairs.at(stairIdx).planeLabels.size(); treadIdx++)
				{
					usedPlanes.insert(stairs.at(stairIdx).planeLabels.at(treadIdx));
				}

				for(int nextStairs = stairIdx+1; nextStairs < stairs.size(); nextStairs++)
				{
					delIDXIter = delIDX.find(nextStairs);
					if(delIDXIter == delIDX.end())
					{
						for(int planeIdx = 0; planeIdx < stairs.at(nextStairs).planeLabels.size(); planeIdx++)
						{
							std::set<int>::iterator foundIndicator;
							foundIndicator = usedPlanes.find(stairs.at(nextStairs).planeLabels.at(planeIdx));
							if(foundIndicator != usedPlanes.end())
							{
								delIDX.insert(nextStairs);
							}
						}
					}
				}
			}
		}
	}
	else
	{
		for(int stairIdx = 0; stairIdx < stairs.size(); stairIdx++) {
			finalize(stairs.at(stairIdx));
			stairSolution.push_back(stairs.at(stairIdx));
		}
	}

	output = stairSolution;
}

void recognition::finalize(Stairs& input)
{
	// #### Change ### //
	// Levenberg-Marquandt optimizer //

	stairPos = input.pos;
	startSearchPoint = stairPos;
	distVec = input.dir;

	stepAmount[0] = input.stairOffset;
	stepAmount[1] = input.stairCount;

	stairRises = input.stairRises;
	stairTreads = input.stairTreads;

	if(optimizeFlag)
	{
		optimizeCoefficients();
//		expandSearch();
	}


	// Find Location of the first stairs //
	int stairOffset = round(stairPos[2]/distVec[2]);
	if(stairOffset <= 0 || stairOffset + stepAmount[0] -1 > 5)
		stairOffset=1;
	if(basePart < 2)
		if((floorInformation) && not(round(stairPos[2]/distVec[2]) + stepAmount[0] -1 > 5))
			stairPos = stairPos - (stairOffset - 1) * distVec;
		else
		{
			stairPos = stairPos + (stepAmount[0]) * distVec;
		}

	input.pos = stairPos;

	input.stairCount = stepAmount[1] - stepAmount[0] + 1;
	input.stairOffset = stairOffset + stepAmount[0] -1;

//        input.width = maxOffset - minOffset;
	input.dir =  distVec;
	input.anchPoint = 0;

	if(input.stairOffset < 0)
		input.stairOffset = 0;

	// ##### Change end #### //

	        // Find stair width
	        Eigen::Vector3f tempVec;
	        tempVec.head(2) = input.dir.head(2);
	        tempVec[2] = 0;
	        tempVec.normalize();

	        Eigen::Vector3f zVec;
	        zVec << 0,0,1;

	        Eigen::Vector3f widthVec;
	        widthVec = tempVec.cross(zVec);
	        widthVec.normalize();

	        std::vector<float> maxOffsetVec;
	        std::vector<float> minOffsetVec;

	        float maxOffset;
	        float minOffset;

	        Eigen::Vector3f stairPos = input.pos;

	        for(int partIdx = 0; partIdx < input.stairRises.size(); partIdx++)
	        {
	        	if(not(widthFlag) || optimizeFlag)
	        		input.stairRises.at(partIdx).getWidth(stairPos,widthVec);
	//            float partWidth = stairs.at(stairCount).stairParts.at(partIdx).width[1] - stairs.at(stairCount).stairParts.at(partIdx).width[0];
	            minOffsetVec.push_back(input.stairRises.at(partIdx).width[0]);
	            maxOffsetVec.push_back(input.stairRises.at(partIdx).width[1]);
	        }
	        for(int partIdx = 0; partIdx < input.stairTreads.size(); partIdx++)
	        {
	        	if(not(widthFlag) || optimizeFlag)
	        		input.stairTreads.at(partIdx).getWidth(stairPos,widthVec);
	//            float partWidth = stairs.at(stairCount).stairParts.at(partIdx).width[1] - stairs.at(stairCount).stairParts.at(partIdx).width[0];
	            minOffsetVec.push_back(input.stairTreads.at(partIdx).width[0]);
	            maxOffsetVec.push_back(input.stairTreads.at(partIdx).width[1]);
	        }

	        sort(minOffsetVec.begin(), minOffsetVec.end());
	        sort(maxOffsetVec.begin(), maxOffsetVec.end());
	        if(optimizeFlag)
	        {
				if(fabs(minOffsetVec[0] - minOffsetVec[2]) < fabs(minOffsetVec[2] - minOffsetVec[4]))
					minOffset = minOffsetVec[0];
				else minOffset = minOffsetVec[2];
				if(fabs(maxOffsetVec[maxOffsetVec.size()-1] - maxOffsetVec[maxOffsetVec.size()-3]) < fabs(maxOffsetVec[maxOffsetVec.size()-3] - maxOffsetVec[maxOffsetVec.size()-5]))
					maxOffset = maxOffsetVec[maxOffsetVec.size()-1];
				else maxOffset = maxOffsetVec[maxOffsetVec.size()-3];
	        }
	        else
	        {
				if(fabs(minOffsetVec[0] - minOffsetVec[1]) < fabs(minOffsetVec[1] - minOffsetVec[2]))
					minOffset = minOffsetVec[0];
				else minOffset = minOffsetVec[1];
				if(fabs(maxOffsetVec[maxOffsetVec.size()-1] - maxOffsetVec[maxOffsetVec.size()-2]) < fabs(maxOffsetVec[maxOffsetVec.size()-2] - maxOffsetVec[maxOffsetVec.size()-3]))
					maxOffset = maxOffsetVec[maxOffsetVec.size()-1];
				else maxOffset = maxOffsetVec[maxOffsetVec.size()-2];
	        }

	        if(not(input.isCircular))
	        	stairPos += widthVec * maxOffset;
	        else {
	        	std::cout<<maxOffset<<std::endl;
	        	std::cout<<input.stairParts.at(0).width[1]<<std::endl;
	        	input.widthOff = maxOffset;
	        }

	        // Use bottom right corner as first anchor Point
	        input.pos = stairPos;

	        input.width = maxOffset - minOffset;
	        if(not(input.isCircular))
	        	findStairRail(input);
	        else
	        	findStairRailCirc(input);
}

void recognition::filterCircular()
{
	wtf=false;
	basePart = 2;
	//// Change for sole circular stairway detection ////
	stairCount = 0;

    for(int firstSeg=0; firstSeg<stairRiseRegions.size()-1; firstSeg++)
    {
    	// Check if the current segment is already used within a solution if yes skip based on the selection //
    	if(graphMeth == false)
    	{
    		if(std::find(globalAddedLabel.begin(), globalAddedLabel.end(), stairRiseRegions.at(firstSeg).segmentLabel) != globalAddedLabel.end())
    		{
    			continue;
    		}
    	}
        for(int secondSeg=firstSeg+1; secondSeg<stairRiseRegions.size();secondSeg++)
        {
        	if(graphMeth == false)
        	{
        		if(std::find(globalAddedLabel.begin(), globalAddedLabel.end(), stairRiseRegions.at(secondSeg).segmentLabel) != globalAddedLabel.end())
        		{
        			continue;
        		}
        	}
        	testBool = false;
//        	if(firstSeg==6 && secondSeg==15)
//        		testBool=true;
//        	else
//        		continue;
            segmentPatch firstPatch = stairRiseRegions.at(firstSeg);
            segmentPatch secondPatch = stairRiseRegions.at(secondSeg);

            if(firstPatch.segmentCentroid[2] > secondPatch.segmentCentroid[2]) {
                firstPatch = stairRiseRegions.at(secondSeg);
                secondPatch = stairRiseRegions.at(firstSeg);
            }



            // Check if planes are close enough - Additional height prerequisites (not necessary) //
            if((firstPatch.segmentCentroid - secondPatch.segmentCentroid).norm() <1 && firstPatch.segmentCentroid[2] < 2.5)
            {
				Eigen::Vector3f dirVec;

				Eigen::Vector2f n1 = firstPatch.segmentCoefficient.head(2);
				n1.normalize();
				Eigen::Vector2f n2 = secondPatch.segmentCoefficient.head(2);
				n2.normalize();

				angleDiff = acos(fabs(n1.dot(n2)))/M_PI*180;

				if(angleDiff > 10 && angleDiff < 50)
				{
					// Check vertical both vertical distances //
					for(int heightInitMode = 0; heightInitMode < 2; heightInitMode++)
					{
						firstPatch.getHeight();
						secondPatch.getHeight();
						firstPatch.getMaxHeight();
						secondPatch.getMaxHeight();

						float vDist;
						if(heightInitMode == 0)
							vDist = (secondPatch.segmentCentroid[2] + secondPatch.height[1]) - (firstPatch.segmentCentroid[2] + firstPatch.height[1]);
						else if(heightInitMode == 1)
							vDist = (secondPatch.segmentCentroid[2] + secondPatch.height[0]) - (firstPatch.segmentCentroid[2] + firstPatch.height[0]);

						if(fabs(vDist) > nDistance[0] && fabs(vDist) < nDistance[1])
						{

							if(n1.dot(n2) > 0)
								distVec.head(2)=n1+n2;
							else
								distVec.head(2)=n1-n2;
							distVec.head(2).normalize();


							// Calculate the center of stairway //
							Eigen::Vector2f l1;
							l1 = distVec.head(2);
							Eigen::Vector2f p1 = firstPatch.segmentCentroid.head(2);

							Eigen::Vector2f l2;
							l2[0] = n2[1];
							l2[1] = -n2[0];
							Eigen::Vector2f p2 = secondPatch.segmentCentroid.head(2);
							//  Change 170703  //

							Eigen::Vector2f intersectPoint;
							intersectPoint[0] = ((p1[0]*(p1[1]-l1[1])-p1[1]*(p1[0]-l1[0]))*l2[0] - (p2[0]*(p2[1]-l2[1])-p2[1]*(p2[0]-l2[0]))*l1[0]) / (l1[0] * l2[1] - l1[1] * l2[0]);
							intersectPoint[1] = ((p1[0]*(p1[1]-l1[1])-p1[1]*(p1[0]-l1[0]))*l2[1] - (p2[0]*(p2[1]-l2[1])-p2[1]*(p2[0]-l2[0]))*l1[1]) / (l1[0] * l2[1] - l1[1] * l2[0]);

							startSearchPoint.head(2) = firstPatch.segmentCentroid.head(2);
							if(heightInitMode == 0)
								startSearchPoint[2] = firstPatch.segmentCentroid[2] + firstPatch.height[1];
							else if(heightInitMode == 1)
								startSearchPoint[2] = firstPatch.segmentCentroid[2] + firstPatch.height[0] + fabs(vDist);

							// TEMPORARILY DISTANCE BETWEEN CENTROIDS //
//							distVec.head(2) = (secondPatch.segmentCentroid.head(2)-startSearchPoint.head(2));
//							float distFactor = ((secondPatch.segmentCentroid.head(2)-startSearchPoint.head(2)).dot(distVec.head(2)));
							float distFactor = (intersectPoint - startSearchPoint.head(2)).norm();
							distVec.head(2)*=distFactor;
							if(vDist > 0) {
								if((secondPatch.segmentCentroid.head(2)-firstPatch.segmentCentroid.head(2)).dot(distVec.head(2)) < 0)
									distVec.head(2) *= -1;
							}
							else if((secondPatch.segmentCentroid.head(2)-firstPatch.segmentCentroid.head(2)).dot(distVec.head(2)) > 0)
								distVec.head(2) *= -1;

							distVec[2] = fabs(vDist);

							if(distVec.head(2).dot(n2) < 0)
								n2 *= -1;
							if(asin(distVec.head(2).dot(n2)) > 0)
								clockWise = false;
							else clockWise = true;


//							distCircCent = (p1[0]-startSearchPoint[0])/l1[0];
//							float distCircCent2 = (p2[0]-startSearchPoint[0])/l2[0];
//
//							if(fabs(distCircCent - distCircCent2) < 1)
//							{

							if(testBool) {
								std::cout<<"FIRST PATCH "<<std::endl<<firstPatch.segmentCentroid<<std::endl;
								std::cout<<"SECOND: "<<std::endl<<secondPatch.segmentCentroid<<std::endl;
								std::cout<<"DIST VEC: "<<std::endl<<distVec<<std::endl;
							}

							stairTreads.clear();
							stairRises.clear();
							addedLabel.clear();
							stairs.push_back(Stairs());

							Eigen::Vector3f nnorm;
							nnorm.head(2) = distVec.head(2);
							nnorm[2] = 0;
							nnorm.normalize();
						    Eigen::Vector3f zVec;
						    zVec << 0,0,1;
						    Eigen::Vector3f widthVec;
						    widthVec = nnorm.cross(zVec);
						    widthVec.normalize();
						    firstPatch.getWidth(startSearchPoint,widthVec);

							addedLabel.push_back(firstPatch.segmentLabel);
							stairs.at(stairCount).stairRiseCloud+=firstPatch.segmentCloud;
							stairs.at(stairCount).stairParts.push_back(firstPatch);
							stairs.at(stairCount).stairRises.push_back(firstPatch);
							stairs.at(stairCount).stairRises.at(stairs.at(stairCount).stairRises.size()-1).segmentLabel=0;
							stairs.at(stairCount).isCircular = true;
							stairs.at(stairCount).angleDiff = angleDiff;
							stairs.at(stairCount).clockwise = clockWise;
							stairRises.push_back(firstPatch);
							stairRises.at(stairRises.size()-1).segmentLabel=0;

							find();
							check();
//							}
						}
					}
				}
            }
        }
    }
	basePart = 3;
}

void recognition::check()
{
    Eigen::Vector3f dirVecNorm = distVec;
    dirVecNorm.normalize();

    //Change for circ stairs //
    if(stairs.at(stairCount).stairParts.size() > 3 &&  ((90 - (acos(fabs(dirVecNorm[2]))/M_PI*180) < 50 && 90 - (acos(fabs(dirVecNorm[2]))/M_PI*180) > 20) || basePart>1) && stepAmount[1] - stepAmount[0] > 2)
    {

    	stairs.at(stairCount).planeLabels = addedLabel;


        Eigen::Matrix<float,5,1> stairScore = getStairScoreNew();

        float accuracy = stairScore[0] * stairScore[1] * stairScore[2] * stairScore[3] * stairScore[4];

        stairs.at(stairCount).pos = stairPos;

        stairs.at(stairCount).stairCount = stepAmount[1];
        stairs.at(stairCount).stairOffset = stepAmount[0];
        stairs.at(stairCount).stairScore = stairScore.head(3);
        stairs.at(stairCount).accuracy = accuracy;

//        stairs.at(stairCount).width = maxOffset - minOffset;
        stairs.at(stairCount).dir =  distVec;
        stairs.at(stairCount).anchPoint = 0;

        if(stairs.at(stairCount).stairOffset < 0)
        	stairs.at(stairCount).stairOffset = 0;
//        std::cout<<(stairs.at(stairCount))<<std::endl;

        for(size_t labCount = 0; labCount < addedLabel.size(); labCount ++)
        	globalAddedLabel.push_back(addedLabel.at(labCount));
//
//        findStairRail();


        stairCount++;
    }
    else
    {
    	// Erase the declined stairs //
        addedLabel.erase(addedLabel.end()-stairs.at(stairCount).stairParts.size(), addedLabel.end());
        stairs.pop_back();
    }
}

Eigen::Vector3f recognition::getStairScore()
{
	Stairs curStair = stairs.at(stairCount);

	Eigen::Vector3f estimateDir;
	estimateDir = distVec;
//	estimateDir += distVec;
	float stairWidth = estimateDir.head(2).norm();

	Eigen::Vector3f estimateNormDir;
	estimateNormDir=estimateDir;
	estimateNormDir[2]=0;
	estimateNormDir.normalize();

	Eigen::Vector3f estStairPos;
	estStairPos = startSearchPoint;

	Eigen::Vector3f errorVec;
	errorVec << 0,0,0;

	Eigen::Vector2i mmSize;
	mmSize[0]=99;
	mmSize[1]=-99;

	if(basePart == 0 || basePart == 1)
	{
		for(size_t stairRiseIdx = 0; stairRiseIdx < curStair.stairRises.size(); stairRiseIdx++)
		{
			// Check horizontal distance //
			segmentPatch tempPatch = stairRises.at(stairRiseIdx);
			float posErr;
			float coeffErr;

			if(tempPatch.segmentLabel > mmSize[1])
				 mmSize[1] = tempPatch.segmentLabel;
			if(tempPatch.segmentLabel < mmSize[0])
				 mmSize[0] = tempPatch.segmentLabel;
			posErr = fabs(((estStairPos.head(2) + tempPatch.segmentLabel * estimateDir.head(2)) - tempPatch.segmentCentroid.head(2)).dot(estimateNormDir.head(2)));

			posErr = (stairWidth-posErr)/stairWidth;

			errorVec[0] += posErr;

			Eigen::Vector3f segCoeffNorm = tempPatch.segmentCoefficient.head(3);

			segCoeffNorm[2] = 0;
			segCoeffNorm.normalize();
			float angError = (fabs(segCoeffNorm.head(2).dot(estimateNormDir.head(2))));
			angError = acos(angError)/M_PI*180;
	//		std::cout<<angError<<std::endl;
	//		if(angError > 10)
	//			std::cout<<"Oh... "<<angError<<std::endl;
			angError = 1-(angError / 20);
			errorVec[1] += angError;
		}

		if(curStair.stairRises.size() < 2 || mmSize[1]-mmSize[0] <2)
		{
			errorVec[0]=0.5;
			errorVec[1]=0.5;
		}
		else
		{
			errorVec[0] /= curStair.stairRises.size();
			errorVec[1] /= curStair.stairRises.size();
			errorVec[1] *= errorVec[1];
		}

		if(errorVec[0] == 1)
			errorVec[0] = 0.5;
		if(errorVec[1] == 1)
			errorVec[1] = 0.5;


		mmSize[0]=99;
		mmSize[1]=-99;

		float extensionFactor =0;

		for(size_t stairTreadIdx = 0; stairTreadIdx < curStair.stairTreads.size(); stairTreadIdx++)
		{
			// Check vertical distance //
			segmentPatch tempPatch = stairTreads.at(stairTreadIdx);
			if(tempPatch.segmentLabel > mmSize[1])
				 mmSize[1] = tempPatch.segmentLabel;
			if(tempPatch.segmentLabel < mmSize[0])
				 mmSize[0] = tempPatch.segmentLabel;
			float posErr;
			float coeffErr;
			posErr = fabs((estStairPos[2] + tempPatch.segmentLabel * estimateDir[2]) - tempPatch.segmentCentroid[2]);
	//		cout<<posErr<<"   "<<estimateVector[2]<<"  "<<stairPos[2]<<" - "<<tempPatch.segmentCentroid[2]<<"   "<<tempPatch.segmentLabel * (distVec[2]+estimateDir[2]) <<endl;;

			posErr = (estimateDir[2]-posErr)/estimateDir[2];

			errorVec[2] += posErr;

			bool checkExtension = false;
			if(curStair.stairRises.size() < 3)
				checkExtension = true;
			if(checkExtension){
				float hDist = (tempPatch.segmentCentroid.head(2) - estStairPos.head(2)).dot(estimateNormDir.head(2)); // Horizontal distance to center
				tempPatch.getTreadDepth(estimateNormDir);
				float updateCut = 0;
				float updateExtend = 0;

				if(tempPatch.segmentLabel >= 0)
				{
					if( hDist + tempPatch.depth[1] > (tempPatch.segmentLabel+1)*stairWidth ) // Vertical distance to stair top
					{
						updateExtend =  hDist + tempPatch.depth[1] - (tempPatch.segmentLabel+1)*stairWidth;
					}
					// Check if stair height too small for ascending or too large for descending
					if( hDist + tempPatch.depth[0] < (tempPatch.segmentLabel)*stairWidth ) // Vertical distance to stair bottom
					{
						updateCut = hDist + tempPatch.depth[0] - (tempPatch.segmentLabel)*stairWidth;
					}
				}
				else
				{
					if( hDist + tempPatch.depth[1] > (tempPatch.segmentLabel+1)*stairWidth ) // Vertical distance to stair top
					{
						updateCut = (tempPatch.segmentLabel+1)*stairWidth -  (hDist + tempPatch.depth[1]);
					}
					// Check if stair height too small for ascending or too large for descending
					if( hDist + tempPatch.depth[0] < (tempPatch.segmentLabel)*stairWidth ) // Vertical distance to stair bottom
					{
						updateExtend = (tempPatch.segmentLabel)*stairWidth - (hDist + tempPatch.depth[0]) ;
					}
				}
				///////// Change 170627 /////////
		//		errorVec[srises*srcals+stairTreadIdx*stcalcs+2]=updateCut+updateCut;
				extensionFactor+=1-(fabs(updateCut)+fabs(updateExtend));
			}
			else
				extensionFactor=curStair.stairTreads.size();
		}

		if(curStair.stairTreads.size() < 2 || mmSize[1]-mmSize[0] <2 || errorVec[2] ==1)
		{
			errorVec[2] = 0.5;
		}
		else
		{
			extensionFactor /= curStair.stairTreads.size();
			errorVec[2] /= curStair.stairTreads.size();
			errorVec[2] *= extensionFactor;
		}
	}
	else if(basePart == 2 || basePart == 3)
		return getStairScoreCirc();
	return errorVec;
}




Eigen::Matrix<float,5,1> recognition::getStairScoreNew()
{
//	Stairs curStair = stairs.at(stairCount);

	Eigen::Vector3f estimateDir;
	estimateDir = distVec;
//	estimateDir += distVec;
	float stairWidth = estimateDir.head(2).norm();

	Eigen::Vector3f estimateNormDir;
	estimateNormDir=estimateDir;
	estimateNormDir[2]=0;
	estimateNormDir.normalize();

	Eigen::Vector3f estStairPos;
	estStairPos = startSearchPoint;

	Eigen::Matrix<float,5,1> errorVec;
	errorVec << 0,0,0,0,0;

	Eigen::Vector2i mmSize;

	int riserAmount = 0;
	int treadAmount = 0;

	int minLab = 90;
	int maxLab = -90;

	for(int partIdx = 0; partIdx < stairRises.size(); partIdx++)
	{
		if( stairRises.at(partIdx).segmentLabel<minLab)
			minLab =  stairRises.at(partIdx).segmentLabel;
		if( stairRises.at(partIdx).segmentLabel>maxLab)
			maxLab =  stairRises.at(partIdx).segmentLabel;
	}
	for(int partIdx = 0; partIdx <  stairTreads.size(); partIdx++)
	{
		if( stairTreads.at(partIdx).segmentLabel<minLab)
			minLab =  stairTreads.at(partIdx).segmentLabel;
		if( stairTreads.at(partIdx).segmentLabel>maxLab)
			maxLab =  stairTreads.at(partIdx).segmentLabel;
	}
	if(basePart == 0 || basePart == 1)
	{
		for(int stepIdx = minLab; stepIdx <= maxLab; stepIdx ++)
		{
			regions currRisers;
			segmentPatch currRiserPatch;
			for(int stairRiseIdx = 0; stairRiseIdx <  stairRises.size(); stairRiseIdx++)
			{
				if( stairRises.at(stairRiseIdx).segmentLabel == stepIdx)
				{
					currRiserPatch.segmentCloud+= stairRises.at(stairRiseIdx).segmentCloud;
					currRisers.push_back( stairRises.at(stairRiseIdx));
				}
			}
			regions currTreads;
			segmentPatch currTreadPatch;
			for(int stairTreadIdx = 0; stairTreadIdx <  stairTreads.size(); stairTreadIdx++)
			{
				if( stairTreads.at(stairTreadIdx).segmentLabel == stepIdx)
				{
					currTreadPatch.segmentCloud+= stairTreads.at(stairTreadIdx).segmentCloud;
					currTreads.push_back( stairTreads.at(stairTreadIdx));
				}
			}

/*#### 		Get score for the riser 		####*/

			if(currRisers.size() >0)
			{
				riserAmount++;
				float posErr;
				float coeffErr;
				currRiserPatch.analyse();
				posErr = fabs(((estStairPos.head(2) + stepIdx * estimateDir.head(2)) - currRiserPatch.segmentCentroid.head(2)).dot(estimateNormDir.head(2)));
				posErr = (stairWidth-posErr)/stairWidth;
				errorVec[0] += posErr;
				Eigen::Vector3f segCoeffNorm = currRiserPatch.segmentCoefficient.head(3);
				segCoeffNorm[2] = 0;
				segCoeffNorm.normalize();
				float angError = (fabs(segCoeffNorm.head(2).dot(estimateNormDir.head(2))));
				if(angError > 0.99999)
					angError = 0.99999;
				angError = acos(angError)/M_PI*180;
				angError = 1-(angError / 20);
				errorVec[1] += angError;

				// Check vertical distance //
				// For ascending stairs - Check top //
				currRiserPatch.getHeight();
				posErr = fabs((estStairPos[2] + stepIdx * estimateDir[2]) - (currRiserPatch.segmentCentroid[2] + currRiserPatch.height[1]));
				posErr = (estimateDir[2]-posErr)/estimateDir[2];
				errorVec[2] += posErr;
				// For other stairs //
			}

/*#### 		Get score for the tread 		####*/

			if(currTreads.size() >0)
			{
				treadAmount++;
				float posErr;
				float coeffErr;
				currTreadPatch.analyse();
				posErr = fabs((estStairPos[2] + stepIdx * estimateDir[2]) - currTreadPatch.segmentCentroid[2]);
		//		cout<<posErr<<"   "<<estimateVector[2]<<"  "<<stairPos[2]<<" - "<<tempPatch.segmentCentroid[2]<<"   "<<tempPatch.segmentLabel * (distVec[2]+estimateDir[2]) <<endl;;
				posErr = (estimateDir[2]-posErr)/estimateDir[2];
				errorVec[3] += posErr;

				// Check horizontal distance //
				// For descending stairs //
				// For other stairs //

				float hDist = (currTreadPatch.segmentCentroid.head(2) - estStairPos.head(2)).dot(estimateNormDir.head(2)); // Horizontal distance to center
				currTreadPatch.getTreadDepth(estimateNormDir);
				float updateCut = 0;
				float updateExtend = 0;

				if(stepIdx >= 0)
				{
					if( hDist + currTreadPatch.depth[1] > (stepIdx+1)*stairWidth ) // Vertical distance to stair top
					{
						updateExtend =  (hDist + currTreadPatch.depth[1]) - (stepIdx+1)*stairWidth;
					}
					// Check if stair height too small for ascending or too large for descending
					if( hDist + currTreadPatch.depth[0] < (stepIdx)*stairWidth ) // Vertical distance to stair bottom
					{
						updateCut = hDist + currTreadPatch.depth[0] - (stepIdx)*stairWidth;
					}
				}
				else
				{
					if( hDist + currTreadPatch.depth[1] > (stepIdx+1)*stairWidth ) // Vertical distance to stair top
					{
						updateCut = (stepIdx+1)*stairWidth -  (hDist + currTreadPatch.depth[1]);
					}
					// Check if stair height too small for ascending or too large for descending
					if( hDist + currTreadPatch.depth[0] < (stepIdx)*stairWidth ) // Vertical distance to stair bottom
					{
						updateExtend = (stepIdx)*stairWidth - (hDist + currTreadPatch.depth[0]) ;
					}
				}
				///////// Change 170627 /////////
		//		errorVec[srises*srcals+stairTreadIdx*stcalcs+2]=updateCut+updateCut;

				if(false && stairRises.size() < 3)
					errorVec[4]+=1-(fabs(updateCut)+fabs(updateExtend));
				else
					errorVec[4]+=1;

			}
		}
	}

	if(riserAmount < 2)
	{
		errorVec[0]=0.5;
		errorVec[1]=0.5;
	}
	else
	{
		errorVec[0] /= riserAmount;
		errorVec[1] /= riserAmount;
		errorVec[2] /= riserAmount;
//		errorVec[1] *= errorVec[1];
	}
	if(treadAmount < 2 )
	{
		errorVec[3] = 0.5;
		errorVec[4] = 0.5;
	}
	else
	{
		errorVec[3] /= treadAmount;
		errorVec[4] /= treadAmount;
//		errorVec[3] *= extensionFactor;
	}

	if(errorVec[0] == 1)
		errorVec[0] = 0.5;
	if(errorVec[1] == 1)
		errorVec[1] = 0.5;
	if(errorVec[2] == 1)
		errorVec[2] = 0.5;
	if(errorVec[3] == 1)
		errorVec[3] = 0.5;
//	if(errorVec[4] == 1)
//		errorVec[4] = 0.5;

//	else if(basePart == 2 || basePart == 3)
//		return getStairScoreCirc();
	return errorVec;
}



Eigen::Vector3f recognition::getStairScoreCirc()
{
	Stairs curStair = stairs.at(stairCount);

	Eigen::Vector3f estimateDir;
	estimateDir = distVec;
//	estimateDir += distVec;
	float stairWidth = estimateDir.head(2).norm();

	Eigen::Vector3f estimateNormDir;
	estimateNormDir=estimateDir;
	estimateNormDir[2]=0;
	estimateNormDir.normalize();

	Eigen::Vector3f estStairPos;
	estStairPos = startSearchPoint;

	Eigen::Vector3f errorVec;
	errorVec << 0,0,0;

	Eigen::Vector2i mmSize;
	mmSize[0]=99;
	mmSize[1]=-99;

	for(size_t stairRiseIdx = 0; stairRiseIdx < curStair.stairRises.size(); stairRiseIdx++)
	{
		// Check horizontal distance //
		segmentPatch tempPatch = stairRises.at(stairRiseIdx);
		float posErr;
		float coeffErr;

		if(tempPatch.segmentLabel > mmSize[1])
			 mmSize[1] = tempPatch.segmentLabel;
		if(tempPatch.segmentLabel < mmSize[0])
			 mmSize[0] = tempPatch.segmentLabel;

		Eigen::Vector2f projNorm;
		Eigen::Matrix2f rotMatrix;
		rotMatrix(0,0) = cosd(angleDiff*(tempPatch.segmentLabel-0.5));
		if(clockWise)
		{
			rotMatrix(0,1) = sind(angleDiff*(tempPatch.segmentLabel-0.5));
			rotMatrix(1.0) = -sind(angleDiff*(tempPatch.segmentLabel-0.5));
		}
		else
		{
			rotMatrix(0,1) = -sind(angleDiff*(tempPatch.segmentLabel-0.5));
			rotMatrix(1.0) = sind(angleDiff*(tempPatch.segmentLabel-0.5));
		}
		rotMatrix(1,1) = cosd(angleDiff*(tempPatch.segmentLabel-0.5));
		projNorm = distVec.head(2).transpose() * rotMatrix;
		projNorm.normalize();

		Eigen::Vector3f stepRefPos;
		Eigen::Vector2f extensionVec;
		extensionVec << 0,0;
		for(int exIter = 0; exIter < abs(tempPatch.segmentLabel); exIter++) {
			int ext = exIter;
			if(tempPatch.segmentLabel<0){
				ext+=1;
				ext*=-1;
			}
			rotMatrix(0,0) = cosd(angleDiff*ext);
			if(clockWise)
			{
				rotMatrix(0,1) = sind(angleDiff*ext);
				rotMatrix(1.0) = -sind(angleDiff*ext);
			}
			else
			{
				rotMatrix(0,1) = -sind(angleDiff*ext);
				rotMatrix(1.0) = sind(angleDiff*ext);
			}
			rotMatrix(1,1) = cosd(angleDiff*ext);
			if(tempPatch.segmentLabel<0)
				extensionVec -= distVec.head(2).transpose() * rotMatrix;
			else
				extensionVec += distVec.head(2).transpose() * rotMatrix;
		}
		stepRefPos.head(2) = estStairPos.head(2) + extensionVec;

		posErr = fabs((stepRefPos.head(2) - tempPatch.segmentCentroid.head(2)).dot(projNorm));
		posErr = (stairWidth-posErr)/stairWidth;
		errorVec[0] += posErr;
		Eigen::Vector3f segCoeffNorm = tempPatch.segmentCoefficient.head(3);
		segCoeffNorm[2] = 0;
		segCoeffNorm.normalize();
		float angError = (fabs(segCoeffNorm.head(2).dot(projNorm)));
		angError = acos(angError)/M_PI*180;
		angError = 1-(angError / 20);
		errorVec[1] += angError;
	}

	if(curStair.stairRises.size() < 2 || mmSize[1]-mmSize[0] <2)
	{
		errorVec[0]=0.5;
		errorVec[1]=0.5;
	}
	else
	{
		errorVec[0] /= curStair.stairRises.size();
		errorVec[1] /= curStair.stairRises.size();
		errorVec[1] *= errorVec[1];
	}

	if(errorVec[0] == 1)
		errorVec[0] = 0.5;
	if(errorVec[1] == 1)
		errorVec[1] = 0.5;


	mmSize[0]=99;
	mmSize[1]=-99;

	for(size_t stairTreadIdx = 0; stairTreadIdx < curStair.stairTreads.size(); stairTreadIdx++)
	{
		// Check vertical distance //
		segmentPatch tempPatch = stairTreads.at(stairTreadIdx);
		if(tempPatch.segmentLabel > mmSize[1])
			 mmSize[1] = tempPatch.segmentLabel;
		if(tempPatch.segmentLabel < mmSize[0])
			 mmSize[0] = tempPatch.segmentLabel;
		float posErr;
		float coeffErr;
		posErr = fabs((estStairPos[2] + tempPatch.segmentLabel * estimateDir[2]) - tempPatch.segmentCentroid[2]);
//		cout<<posErr<<"   "<<estimateVector[2]<<"  "<<stairPos[2]<<" - "<<tempPatch.segmentCentroid[2]<<"   "<<tempPatch.segmentLabel * (distVec[2]+estimateDir[2]) <<endl;;

		posErr = (estimateDir[2]-posErr)/estimateDir[2];

		errorVec[2] += posErr;
	}

	if(curStair.stairTreads.size() < 2 || mmSize[1]-mmSize[0] <2 || errorVec[2] ==1)
	{
		errorVec[2] = 0.5;
	}
	else
	{
		errorVec[2] /= curStair.stairTreads.size();
	}
	return errorVec;
}

bool recognition::widthReqCirc(segmentPatch& testPatch, int stairNo)
{
	if(not(widthFlag)) {
		return true;
	}
//	double widthStart = pcl::getTime();
	// Check width requirements //
	float width;

	// Staircase and canditate overlap? //
	bool overlapFlag = true;
	// Staircase and candidate together do not exceed the maximum width? //
	bool maxWidthFlag = true;

    Eigen::Vector3f tempVec;
    tempVec.head(2) = distVec.head(2);
    tempVec[2] = 0;
    tempVec.normalize();

	Eigen::Vector3f projNorm;
	Eigen::Matrix2f rotMatrix;
	rotMatrix(0,0) = cosd(angleDiff*(stairNo-0.5));
	if(clockWise)
	{
		rotMatrix(0,1) = sind(angleDiff*(stairNo-0.5));
		rotMatrix(1.0) = -sind(angleDiff*(stairNo-0.5));
	}
	else
	{
		rotMatrix(0,1) = -sind(angleDiff*(stairNo-0.5));
		rotMatrix(1.0) = sind(angleDiff*(stairNo-0.5));
	}
	rotMatrix(1,1) = cosd(angleDiff*(stairNo-0.5));
	projNorm.head(2) = distVec.head(2).transpose() * rotMatrix;
	projNorm[2] = 0;
	projNorm.normalize();

	Eigen::Vector3f stepRefPos;
	Eigen::Vector2f extensionVec;
	extensionVec << 0,0;
	for(int exIter = 0; exIter < abs(stairNo); exIter++) {
		int ext = exIter;
		if(stairNo<0){
			ext+=1;
			ext*=-1;
		}
		rotMatrix(0,0) = cosd(angleDiff*ext);
		if(clockWise)
		{
			rotMatrix(0,1) = sind(angleDiff*ext);
			rotMatrix(1.0) = -sind(angleDiff*ext);
		}
		else
		{
			rotMatrix(0,1) = -sind(angleDiff*ext);
			rotMatrix(1.0) = sind(angleDiff*ext);
		}
		rotMatrix(1,1) = cosd(angleDiff*ext);
		if(stairNo<0)
			extensionVec -= distVec.head(2).transpose() * rotMatrix;
		else
			extensionVec += distVec.head(2).transpose() * rotMatrix;
	}
	stepRefPos.head(2) = stairPos.head(2) + extensionVec;

    Eigen::Vector3f zVec;
    zVec << 0,0,1;

    Eigen::Vector3f widthVec;
    widthVec = projNorm.cross(zVec);
    widthVec.normalize();

    std::vector<float> maxOffsetVec;
    std::vector<float> minOffsetVec;

    float maxOffset;
    float minOffset;

    for(int partIdx = 0; partIdx < stairs.at(stairCount).stairParts.size(); partIdx++)
    {
//    	double gwStart = pcl::getTime();
//    	stairs.at(stairCount).stairParts.at(partIdx).getWidth(stairPos,widthVec);
//    	double gwEnd = pcl::getTime();
//    	getWTime += gwEnd - gwStart;
        minOffsetVec.push_back(stairs.at(stairCount).stairParts.at(partIdx).width[0]);
        maxOffsetVec.push_back(stairs.at(stairCount).stairParts.at(partIdx).width[1]);
    }

    double sortStart = pcl::getTime();
    sort(minOffsetVec.begin(), minOffsetVec.end());
    sort(maxOffsetVec.begin(), maxOffsetVec.end());
    double sortEnd = pcl::getTime();
    sortTime += sortEnd - sortStart;
	minOffset = minOffsetVec[0];
	maxOffset = maxOffsetVec[maxOffsetVec.size()-1];

    testPatch.getWidth(stepRefPos,widthVec);

    if(testPatch.width[1] < minOffset+0.2 || testPatch.width[0] > maxOffset-0.2)
    	overlapFlag = false;

    minOffsetVec.push_back(testPatch.width[0]);
    maxOffsetVec.push_back(testPatch.width[1]);

    sortStart = pcl::getTime();
    sort(minOffsetVec.begin(), minOffsetVec.end());
    sort(maxOffsetVec.begin(), maxOffsetVec.end());
    sortEnd = pcl::getTime();
    sortTime += sortEnd - sortStart;
	minOffset = minOffsetVec[0];
	maxOffset = maxOffsetVec[maxOffsetVec.size()-1];

    width = maxOffset - minOffset;

//    double widthEnd = pcl::getTime();
//    widthTime += widthEnd- widthStart;

    if(width > widthReqVec[1])
    	maxWidthFlag = false;

    if(maxWidthFlag && overlapFlag)
    	return true;
    else
    	return false;
}


bool recognition::widthReq(segmentPatch& testPatch)
{
	if(not(widthFlag)) {
		return true;
	}

//	double widthStart = pcl::getTime();
	// Check width requirements //
	float width;

	// Staircase and canditate overlap? //
	bool overlapFlag = true;
	// Staircase and candidate together do not exceed the maximum width? //
	bool maxWidthFlag = true;

    Eigen::Vector3f tempVec;
    tempVec.head(2) = distVec.head(2);
    tempVec[2] = 0;
    tempVec.normalize();

    Eigen::Vector3f zVec;
    zVec << 0,0,1;

    Eigen::Vector3f widthVec;
    widthVec = tempVec.cross(zVec);
    widthVec.normalize();

    std::vector<float> maxOffsetVec;
    std::vector<float> minOffsetVec;

    float maxOffset;
    float minOffset;

    for(int partIdx = 0; partIdx < stairs.at(stairCount).stairParts.size(); partIdx++)
    {
//    	double gwStart = pcl::getTime();
//    	stairs.at(stairCount).stairParts.at(partIdx).getWidth(stairPos,widthVec);
//    	double gwEnd = pcl::getTime();
//    	getWTime += gwEnd - gwStart;
        minOffsetVec.push_back(stairs.at(stairCount).stairParts.at(partIdx).width[0]);
        maxOffsetVec.push_back(stairs.at(stairCount).stairParts.at(partIdx).width[1]);
    }

    double sortStart = pcl::getTime();
    sort(minOffsetVec.begin(), minOffsetVec.end());
    sort(maxOffsetVec.begin(), maxOffsetVec.end());
    double sortEnd = pcl::getTime();
    sortTime += sortEnd - sortStart;
	minOffset = minOffsetVec[0];
	maxOffset = maxOffsetVec[maxOffsetVec.size()-1];

    testPatch.getWidth(stairPos,widthVec);

    if(testPatch.width[1] < minOffset + 0.05 || testPatch.width[0] > maxOffset - 0.05)
    	overlapFlag = false;

    minOffsetVec.push_back(testPatch.width[0]);
    maxOffsetVec.push_back(testPatch.width[1]);

    sortStart = pcl::getTime();
    sort(minOffsetVec.begin(), minOffsetVec.end());
    sort(maxOffsetVec.begin(), maxOffsetVec.end());
    sortEnd = pcl::getTime();
    sortTime += sortEnd - sortStart;
	minOffset = minOffsetVec[0];
	maxOffset = maxOffsetVec[maxOffsetVec.size()-1];

    width = maxOffset - minOffset;

//    double widthEnd = pcl::getTime();
//    widthTime += widthEnd- widthStart;

    if(width > widthReqVec[1])
    	maxWidthFlag = false;

    if(maxWidthFlag && overlapFlag)
    	return true;
    else
    	return false;
}

void recognition::find()
{
	if(inputRegions.size() == 0)
	{
		std::cout<<"No input regions"<<std::endl;
		return;
	}
    inputRegions.generateCenterCloud();
//    if(stairTreadRegions.size() == 0)
//    	stairTreadRegions.push_back(inputRegions.at(0));
//    if(stairRiseRegions.size() == 0)
//    	stairRiseRegions.push_back(inputRegions.at(0));

    stairTreadRegions.generateCenterCloud();
    stairRiseRegions.generateCenterCloud();

    stepAmount[0]=0;
    stepAmount[1]=0;

    float searchRadius = 1;
    int searchExtensions = 3;
    pcl::KdTreeFLANN<PointT> stairRiseTree;
    if(stairRiseRegions.size() > 0)
    	stairRiseTree.setInputCloud (stairRiseRegions.centerCloud.makeShared());

    pcl::KdTreeFLANN<PointT> stairTreadTree;
    if(stairTreadRegions.size() > 0)
    	stairTreadTree.setInputCloud (stairTreadRegions.centerCloud.makeShared());

    stairPos = startSearchPoint;

    int direction = 0;

    for(int directionIdx =0; directionIdx <3; directionIdx++)
    {
        if(directionIdx==1)
            direction=-1;
        if(directionIdx==2)
            direction=1;

        int maxExtend=searchExtensions;
        for(int extension=1; extension <=maxExtend; extension++)
        {
        	PointT searchPoint;
        	if(basePart < 2)
        	{
				Eigen::Vector3f searchPointVec = startSearchPoint+direction*extension*distVec;
				searchPoint.x=searchPointVec[0];
				searchPoint.y=searchPointVec[1];
				searchPoint.z=searchPointVec[2];
        	}
        	else
        	{
        		Eigen::Vector2f extensionVec;
        		extensionVec << 0,0;
        		for(int exIter = 0; exIter < abs(direction*extension); exIter++) {
        			int ext = exIter;
        			if(direction<0) {
        				ext+=1;
        			}
					Eigen::Matrix2f rotMatrix;
					rotMatrix(0,0) = cosd(angleDiff*direction*ext);
					if(clockWise)
					{
						rotMatrix(0,1) = sind(angleDiff*direction*ext);
						rotMatrix(1.0) = -sind(angleDiff*direction*ext);
					}
					else
					{
						rotMatrix(0,1) = -sind(angleDiff*direction*ext);
						rotMatrix(1.0) = sind(angleDiff*direction*ext);
					}
					rotMatrix(1,1) = cosd(angleDiff*direction*ext);
					if(direction>0)
						extensionVec += distVec.head(2).transpose() * rotMatrix;
					else
						extensionVec -= distVec.head(2).transpose() * rotMatrix;
        		}

        		Eigen::Vector3f searchPointVec;
        		searchPointVec.head(2) = startSearchPoint.head(2) + extensionVec;
        		searchPointVec[2] = startSearchPoint[2]+direction*extension*distVec[2] ;
				searchPoint.x=searchPointVec[0];
				searchPoint.y=searchPointVec[1];
				searchPoint.z=searchPointVec[2];
//        		std::cout<<"For an extension of "<<direction*extension<<" it is:"<<std::endl;
//            	std::cout<<"Looking at: "<<searchPointVec<<std::endl;
//            	std::cout<<"Extension Vector: "<<extensionVec<<"   "<<direction*extension*distVec[2]<<std::endl;
        	}

        	bool extend = false;
            // Check for stair risers //
        	if(stairRiseRegions.size() > 0)
        	{
				std::vector<int> pointIdxStairRise;
				std::vector<float> squaredDistStairRise;


				if ( stairRiseTree.radiusSearch (searchPoint, searchRadius, pointIdxStairRise, squaredDistStairRise) > 0 )
				{
					for (size_t foundPoints = 0; foundPoints < pointIdxStairRise.size (); foundPoints++)
					{
						if(isStairRiseMatch(pointIdxStairRise.at(foundPoints),direction*extension))
						{
							if(stepAmount[0] > direction*extension)
								stepAmount[0] = direction*extension;
							if(stepAmount[1] < direction*extension)
								stepAmount[1] = direction*extension;
							extend = true;
						}
					}

				}
        	}

            // Check for stair treads //
        	if(stairTreadRegions.size() > 0)
        	{
				std::vector<int> pointIdxStairTread;
				std::vector<float> squaredDistStairTread;
				if ( stairTreadTree.radiusSearch (searchPoint, searchRadius, pointIdxStairTread, squaredDistStairTread) > 0 )
				{
					for (size_t foundPoints = 0; foundPoints < pointIdxStairTread.size (); foundPoints++)
					{
	//                	std::cout<<"Point "<<foundPoints<<" out of "<<pointIdxStairTread.size ()<<std::endl;
						if(isStairTreadMatch(pointIdxStairTread.at(foundPoints),direction*extension))
						{
							if(stepAmount[0] > direction*extension)
								stepAmount[0] = direction*extension;
							if(stepAmount[1] < direction*extension)
								stepAmount[1] = direction*extension;
							extend=true;
						}
					}
				}
        	}
            // Extend if sth. was found //
            if(extend)
                maxExtend=extension+searchExtensions;
            if(direction == 0)
                extension = maxExtend;

        }
    }
}

bool recognition::isStairRiseMatch(int regPos, int stairNo)
{
	if(graphMeth == true)
	{
		if(std::find(globalAddedLabel.begin(), globalAddedLabel.end(), stairRiseRegions.at(regPos).segmentLabel) != globalAddedLabel.end())
		{
			return false;
		}
	}

	double add_start = pcl::getTime();
    segmentPatch candidate = stairRiseRegions.at(regPos);
    if(not(candidate.segmentCentroid[2] > 0.10 || not(floorInformation)))
    	return false;
    Eigen::Vector3f normTemp;
    normTemp = candidate.segmentCoefficient.head(3);
    normTemp[2] = 0;
    normTemp.normalize();

    if(basePart == 0) // Base part = stair rise
    {
//        float vDist = candidate.maxHeight - startSearchPoint[2]; // Vertical distance top of stairrise to top of stairrise
        float vDist = candidate.segmentCentroid[2] - startSearchPoint[2]; // Vertical distance center of stairrise to top of stairrise
        Eigen::Vector2f normDirVec = distVec.head(2);
        float stairDepth = distVec.head(2).norm();      // Stair depth
        normDirVec.normalize();
        float hDist = (candidate.segmentCentroid.head(2) - startSearchPoint.head(2)).dot(normDirVec); // Horizontal distance to center

//        if(fabs(vDist - stairNo * distVec[2]) < maxStairRiseHDist) // Check // Vertical distance top of stairrise to top of stairrise
        if(vDist + 0.04 > (stairNo-1) * distVec[2] && vDist - 0.04 < (stairNo) * distVec[2]  && stairNo != 0) // Check vertical distance
        {
            if(fabs( hDist - stairNo*stairDepth) < maxStairTreadDist ) // Check the horizontal difference
            {
                if(acos(fabs(normTemp.head(2).dot(normDirVec))) / M_PI *180 < 20) // Check the angle difference
                {
                	if(widthReq(candidate))
                	{						if(normDirVec.dot(normTemp.head(2)) < 0)
							normTemp=-normTemp; // Check for 180° rotation simalarity

						if(updateFlag)
						{

							Eigen::Vector3f widthDir; // Vector for stair width calculation
							widthDir.head(2)=normDirVec;
							widthDir[2] = 0;
							candidate.getHeight();

							float updateCut = 0;
							float updateExtend = 0;

							if(stairNo >= 0)
							{
								if( vDist + candidate.height[1] > stairNo*distVec[2] ) // Vertical distance to stair top
								{
									updateExtend =  vDist + candidate.height[1] - stairNo*distVec[2];
								}
								// Check if stair height too small for ascending or too large for descending
								if( vDist + candidate.height[0] < (stairNo-1)*distVec[2] ) // Vertical distance to stair bottom
								{
									updateCut = vDist + candidate.height[0] - (stairNo-1)*distVec[2];
								}
							}
							else
							{
								if( vDist + candidate.height[1] > stairNo*distVec[2] ) // Vertical distance to stair top
								{
									updateCut = stairNo*distVec[2] -  (vDist + candidate.height[1]);
								}
								// Check if stair height too small for ascending or too large for descending
								if( vDist + candidate.height[0] < (stairNo-1)*distVec[2] ) // Vertical distance to stair bottom
								{
									updateExtend = (stairNo-1)*distVec[2] - (vDist + candidate.height[0]) ;
								}
							}

							Eigen::Vector3f updateVector;
							updateVector.head(2)=normDirVec * (hDist / stairNo); // Update stair width and direction
							updateVector[2] = distVec[2] + updateCut + updateExtend;

							distVec = (distVec * stairs.at(stairCount).stairParts.size() + updateVector)/(stairs.at(stairCount).stairParts.size()+1);
						}


						addedLabel.push_back(candidate.segmentLabel);
						stairs.at(stairCount).stairParts.push_back(candidate);

						stairs.at(stairCount).stairRiseCloud+=candidate.segmentCloud;
						stairs.at(stairCount).stairRises.push_back(candidate);
						stairs.at(stairCount).stairRises.at(stairs.at(stairCount).stairRises.size()-1).segmentLabel = stairNo;
						stairRises.push_back(candidate);
						stairRises.at(stairRises.size()-1).segmentLabel = stairNo;
						double add_end = pcl::getTime();
						memTime += add_end-add_start;
						return true;
                	}
                }
            }
        }
    }
    else if(basePart==1)
    {
        float vDist = candidate.segmentCentroid[2] - startSearchPoint[2]; // Vertical distance center of stairrise to top of stairrise
        Eigen::Vector2f normDirVec = distVec.head(2);
        float stairWidth = distVec.head(2).norm();      // Stair width
        normDirVec.normalize();
        float hDist = (candidate.segmentCentroid.head(2) - startSearchPoint.head(2)).dot(normDirVec); // Horizontal distance to center

        if(fabs(vDist-stairNo*distVec[2]) < maxStairRiseDist) // Check the vertical difference
        {
            if(fabs( hDist - stairNo*stairWidth) < maxStairRiseDist ) // Check the horizontal distance
            {
                if(acos(fabs(normDirVec.dot(normTemp.head(2)))) / M_PI *180 < 20) // Check the angle difference
                {

//					std::cout<<"I do not understand..."<<acos(fabs(normDirVec.dot(normTemp.head(2)))) / M_PI *180<<std::endl<<std::endl;

                	if(widthReq(candidate))
                	{
						if(normDirVec.dot(normTemp.head(2)) < 0)
							normTemp=-normTemp; // Check for 180° rotation simalarity
						if(updateFlag)
						{
							Eigen::Vector3f updateVector;
							if(stairNo==0)
							{
								updateVector.head(2)=normTemp.head(2) * distVec.head(2).norm();   // Update only direction
								updateVector[2]=distVec[2]; // Don't update height
	//                            std::cout<<"CHANGED HEAD"<<std::endl;
							}
							else
							{
								updateVector.head(2)=normTemp.head(2) * (hDist / stairNo); // Update direction and stair width
								updateVector[2]=distVec[2]; // Don't update height
							}
							distVec = (distVec * stairs.at(stairCount).stairParts.size() + updateVector)/(stairs.at(stairCount).stairParts.size()+1);
						}
						addedLabel.push_back(candidate.segmentLabel);
						stairs.at(stairCount).stairParts.push_back(candidate);

						stairs.at(stairCount).stairRiseCloud+=candidate.segmentCloud;
						stairs.at(stairCount).stairRises.push_back(candidate);
						stairs.at(stairCount).stairRises.at(stairs.at(stairCount).stairRises.size()-1).segmentLabel = stairNo;
						stairRises.push_back(candidate);
						stairRises.at(stairRises.size()-1).segmentLabel = stairNo;
						double add_end = pcl::getTime();
						memTime += add_end-add_start;
						return true;
                	}
                }
            }
        }
    }
    else if(basePart == 2)
    {
        float vDist = candidate.segmentCentroid[2] - startSearchPoint[2]; // Vertical distance center of stairrise to top of stairrise
        if(vDist + 0.04 > (stairNo-1) * distVec[2] && vDist - 0.04 < (stairNo) * distVec[2] ) // && stairNo != 0) // Check vertical distance
        {
    		Eigen::Vector2f candNorm = candidate.segmentCoefficient.head(2);
    		candNorm.normalize();

    		Eigen::Vector2f projNorm;
			Eigen::Matrix2f rotMatrix;
			rotMatrix(0,0) = cosd(angleDiff*(stairNo-0.5));
			if(clockWise)
			{
				rotMatrix(0,1) = sind(angleDiff*(stairNo-0.5));
				rotMatrix(1.0) = -sind(angleDiff*(stairNo-0.5));
			}
			else
			{
				rotMatrix(0,1) = -sind(angleDiff*(stairNo-0.5));
				rotMatrix(1.0) = sind(angleDiff*(stairNo-0.5));
			}
			rotMatrix(1,1) = cosd(angleDiff*(stairNo-0.5));
			projNorm = distVec.head(2).transpose() * rotMatrix;
			projNorm.normalize();
			Eigen::Vector2f tempOut =distVec.head(2);
			tempOut.normalize();

    		if(acos(fabs(candNorm.dot(projNorm)))/M_PI*180 < 5)
    		{

        		Eigen::Vector2f extensionVec;
        		extensionVec << 0,0;
        		for(int exIter = 0; exIter < abs(stairNo); exIter++) {
        			int ext = exIter;
        			if(stairNo<0){
        				ext+=1;
        				ext*=-1;
        			}
    				rotMatrix(0,0) = cosd(angleDiff*ext);
    				if(clockWise)
    				{
    					rotMatrix(0,1) = sind(angleDiff*ext);
    					rotMatrix(1.0) = -sind(angleDiff*ext);
    				}
    				else
    				{
    					rotMatrix(0,1) = -sind(angleDiff*ext);
    					rotMatrix(1.0) = sind(angleDiff*ext);
    				}
    				rotMatrix(1,1) = cosd(angleDiff*ext);
    				if(stairNo<0)
    					extensionVec -= distVec.head(2).transpose() * rotMatrix;
    				else
    					extensionVec += distVec.head(2).transpose() * rotMatrix;
        		}

        		Eigen::Vector2f searchPointVec;
        		searchPointVec = startSearchPoint.head(2) + extensionVec;

//    			float candDist = (candidate.segmentCentroid[0]-startSearchPoint[0])/candLine[0];
    			if(fabs((searchPointVec - candidate.segmentCentroid.head(2)).dot(projNorm)) < 0.1)
    			{
    				if(widthReqCirc(candidate,stairNo)) {
						addedLabel.push_back(candidate.segmentLabel);
						stairs.at(stairCount).stairParts.push_back(candidate);

						stairs.at(stairCount).stairRiseCloud+=candidate.segmentCloud;
						stairs.at(stairCount).stairRises.push_back(candidate);
						stairs.at(stairCount).stairRises.at(stairs.at(stairCount).stairRises.size()-1).segmentLabel = stairNo;
						stairRises.push_back(candidate);
						stairRises.at(stairRises.size()-1).segmentLabel = stairNo;
						return true;
    				}
    			}
    		}
        }
    }
    return false;
}

bool recognition::isStairTreadMatch(int regPos, int stairNo)
{
	if(graphMeth == true)
	{
		if(std::find(globalAddedLabel.begin(), globalAddedLabel.end(), stairTreadRegions.at(regPos).segmentLabel) != globalAddedLabel.end())
		{
			return false;
		}
	}

	double add_start = pcl::getTime();
    segmentPatch candidate = stairTreadRegions.at(regPos);
    if(not(candidate.segmentCentroid[2] > 0.10 || not(floorInformation)))
    	return false;
    if(basePart== 0)
    {

        float vDist = candidate.segmentCentroid[2] - startSearchPoint[2]; // Vertical distance
        Eigen::Vector2f normDirVec = distVec.head(2);
        float stairWidth = normDirVec.norm();
        normDirVec.normalize();
        float hDist = (candidate.segmentCentroid.head(2) - startSearchPoint.head(2)).dot(normDirVec); // Horizontal distance to center

        if(fabs(vDist - (stairNo) * distVec[2]) < maxStairRiseDist)
        {
            if(hDist > stairNo*stairWidth && hDist < (stairNo+1)*stairWidth) // Check the horizontal distance
            {
            	if(widthReq(candidate))
            	{
					if(updateFlag)
					{
						Eigen::Vector3f updateVector;
						updateVector.head(2)=distVec.head(2); // Don't update direction and stair width - because stair riser basis is more accurate
						if(stairNo == 0)
						{
							updateVector[2]=distVec[2]; // Don't update height for stairtread 0
						}
						else
							updateVector[2]=vDist / stairNo; // Update height
						distVec = (distVec * stairs.at(stairCount).stairParts.size() + updateVector)/(stairs.at(stairCount).stairParts.size()+1);
					}


					addedLabel.push_back(candidate.segmentLabel);

					stairs.at(stairCount).stairTreadCloud+=candidate.segmentCloud;

					stairs.at(stairCount).stairParts.push_back(candidate);
					stairs.at(stairCount).stairTreads.push_back(candidate);
					stairs.at(stairCount).stairTreads.at(stairs.at(stairCount).stairTreads.size()-1).segmentLabel = stairNo;
					stairTreads.push_back(candidate);
					stairTreads.at(stairTreads.size()-1).segmentLabel = stairNo;
					double add_end = pcl::getTime();
					memTime += add_end-add_start;
					return true;
            	}
            }
        }
    }
    else if(basePart==1)
    {
        float vDist = candidate.segmentCentroid[2] - startSearchPoint[2]; // Vertical distance
        Eigen::Vector2f normDirVec = distVec.head(2);
        float stairWidth = normDirVec.norm();      // Stair width
        normDirVec.normalize();
        float hDist = (candidate.segmentCentroid.head(2) - startSearchPoint.head(2)).dot(normDirVec); // Horizontal distance to center

        if(fabs(vDist-stairNo*distVec[2]) < maxStairRiseDist && stairNo!= 0) // Check the vertical difference
        {
            if( hDist > stairNo*stairWidth && hDist < (stairNo+1)*stairWidth ) // Check the horizontal distance
            {
            	if(widthReq(candidate))
            	{
					if(updateFlag) // Update stair slope vector
					{
						// UPDATE FUNCTION DOESN'T WORK PROPERLY //


						Eigen::Vector3f widthDir; // Vector for stair width calculation
						widthDir.head(2)=normDirVec;
						widthDir[2] = 0;
						candidate.getTreadDepth(widthDir);

						float updateCut = 0;
						float updateExtend = 0;

						if(stairNo >= 0)
						{
							if( hDist + candidate.depth[1] > (stairNo+1)*stairWidth ) // Vertical distance to stair top
							{
								updateExtend =  hDist + candidate.depth[1] - (stairNo+1)*stairWidth;
							}
							// Check if stair height too small for ascending or too large for descending
							if( hDist + candidate.depth[0] < (stairNo)*stairWidth ) // Vertical distance to stair bottom
							{
								updateCut = hDist + candidate.depth[0] - (stairNo)*stairWidth;
							}
						}
						else
						{
							if( hDist + candidate.depth[1] > (stairNo+1)*stairWidth ) // Vertical distance to stair top
							{
								updateCut = (stairNo+1)*stairWidth -  (hDist + candidate.depth[1]);
							}
							// Check if stair height too small for ascending or too large for descending
							if( hDist + candidate.depth[0] < (stairNo)*stairWidth ) // Vertical distance to stair bottom
							{
								updateExtend = (stairNo)*stairWidth - (hDist + candidate.depth[0]) ;
							}
						}

						Eigen::Vector3f updateVector;
						updateVector.head(2)=normDirVec * (distVec.head(2).norm() + updateCut + updateExtend); // Update stair width
						updateVector[2]=vDist/stairNo;      // Update height
						distVec = (distVec * stairs.at(stairCount).stairParts.size() + updateVector)/(stairs.at(stairCount).stairParts.size()+1);

					}
					addedLabel.push_back(candidate.segmentLabel);
					stairs.at(stairCount).stairTreadCloud+=candidate.segmentCloud;
					stairs.at(stairCount).stairParts.push_back(candidate);
					stairs.at(stairCount).stairTreads.push_back(candidate);
					stairs.at(stairCount).stairTreads.at(stairs.at(stairCount).stairTreads.size()-1).segmentLabel = stairNo;
					stairTreads.push_back(candidate);
					stairTreads.at(stairTreads.size()-1).segmentLabel = stairNo;
					double add_end = pcl::getTime();
					memTime += add_end-add_start;
					return true;
				}
            }

        }
    }
    else if(basePart==2) {
    	float vDist = candidate.segmentCentroid[2] - startSearchPoint[2]; // Vertical distance center of stairrise to top of stairrise
		if(fabs(vDist-stairNo*distVec[2]) < maxStairRiseDist) // && stairNo!= 0) // Check vertical distance
		{
    		Eigen::Vector2f extensionVec;
    		Eigen::Matrix2f rotMatrix;
    		extensionVec << 0,0;
    		for(int exIter = 0; exIter < abs(stairNo); exIter++) {
    			int ext = exIter;
    			if(stairNo<0){
    				ext+=1;
    				ext*=-1;
    			}
				rotMatrix(0,0) = cosd(angleDiff*ext);
				if(clockWise)
				{
					rotMatrix(0,1) = sind(angleDiff*ext);
					rotMatrix(1.0) = -sind(angleDiff*ext);
				}
				else
				{
					rotMatrix(0,1) = -sind(angleDiff*ext);
					rotMatrix(1.0) = sind(angleDiff*ext);
				}
				rotMatrix(1,1) = cosd(angleDiff*ext);
				if(stairNo<0)
					extensionVec -= distVec.head(2).transpose() * rotMatrix;
				else
					extensionVec += distVec.head(2).transpose() * rotMatrix;
    		}
    		Eigen::Vector2f lowerBord;
    		lowerBord = startSearchPoint.head(2) + extensionVec;

    		extensionVec << 0,0;
    		for(int exIter = 0; exIter < abs(stairNo+1); exIter++) {
    			int ext = exIter;
    			if(stairNo<0){
    				ext+=1;
    				ext*=-1;
    			}
				rotMatrix(0,0) = cosd(angleDiff*ext);
				if(clockWise)
				{
					rotMatrix(0,1) = sind(angleDiff*ext);
					rotMatrix(1.0) = -sind(angleDiff*ext);
				}
				else
				{
					rotMatrix(0,1) = -sind(angleDiff*ext);
					rotMatrix(1.0) = sind(angleDiff*ext);
				}
				rotMatrix(1,1) = cosd(angleDiff*ext);
				if(stairNo<0)
					extensionVec -= distVec.head(2).transpose() * rotMatrix;
				else
					extensionVec += distVec.head(2).transpose() * rotMatrix;

    		}
			Eigen::Vector2f upperBord;
			upperBord = startSearchPoint.head(2) + extensionVec;


			rotMatrix(0,0) = cosd(angleDiff*(stairNo - 0.5));
			if(clockWise)
			{
				rotMatrix(0,1) = sind(angleDiff*(stairNo - 0.5));
				rotMatrix(1.0) = -sind(angleDiff*(stairNo - 0.5));
			}
			else
			{
				rotMatrix(0,1) = -sind(angleDiff*(stairNo - 0.5));
				rotMatrix(1.0) = sind(angleDiff*(stairNo - 0.5));
			}
			rotMatrix(1,1) = cosd(angleDiff*(stairNo - 0.5));
			Eigen::Vector2f projNormUpper;
			projNormUpper = distVec.head(2).transpose() * rotMatrix;
			projNormUpper.normalize();

			rotMatrix(0,0) = cosd(angleDiff*(stairNo + 0.5));
			if(clockWise)
			{
				rotMatrix(0,1) = sind(angleDiff*(stairNo + 0.5));
				rotMatrix(1.0) = -sind(angleDiff*(stairNo + 0.5));
			}
			else
			{
				rotMatrix(0,1) = -sind(angleDiff*(stairNo + 0.5));
				rotMatrix(1.0) = sind(angleDiff*(stairNo + 0.5));
			}
			rotMatrix(1,1) = cosd(angleDiff*(stairNo));
			Eigen::Vector2f projNormLower;
			projNormLower = distVec.head(2).transpose() * rotMatrix;
			projNormLower.normalize();

			if((projNormLower.dot(candidate.segmentCentroid.head(2)-lowerBord)>0 && projNormUpper.dot(candidate.segmentCentroid.head(2)-upperBord)<0))
			{
				if(widthReqCirc(candidate,stairNo)) {
					addedLabel.push_back(candidate.segmentLabel);
					stairs.at(stairCount).stairTreadCloud+=candidate.segmentCloud;
					stairs.at(stairCount).stairParts.push_back(candidate);
					stairs.at(stairCount).stairTreads.push_back(candidate);
					stairs.at(stairCount).stairTreads.at(stairs.at(stairCount).stairTreads.size()-1).segmentLabel = stairNo;
					stairTreads.push_back(candidate);
					stairTreads.at(stairTreads.size()-1).segmentLabel = stairNo;
					return true;
				}
			}
		}
    }
    return false;
}

void recognition::findStairRail(Stairs& input)
{
    Eigen::Vector3f dirVecNorm = input.dir;
    dirVecNorm.normalize();

    Eigen::Vector3f horVec;
    horVec.head(2) = input.dir.head(2);
    horVec[2]=0;
    horVec.normalize();

    int leftCounter = 0;
    int rightCounter = 0;

    Eigen::Vector3f zVec;
    zVec<<0,0,1;
    Eigen::Vector3f wVec = horVec.cross(zVec);

    Eigen::Vector3f posTemp = input.pos;
    Eigen::Vector3f dirTemp = input.dir;

    float vertAngle = 90 - (acos(fabs(dirVecNorm[2])))/M_PI*180;

//    std::cout<<"Stair slope: "<<vertAngle<<std::endl;

    for(int regIdx = 0; regIdx < inputRegions.size(); regIdx++)
    {
    	segmentPatch tempPatch = inputRegions.at(regIdx);
        Eigen::Vector3f mainEV = tempPatch.eigen_vectors.col(2);
        // Look planes with same sloop main EV or z-axis is main EV
        if(acos(fabs(mainEV.dot(dirVecNorm)))/M_PI*180 < 15 || acos(fabs(mainEV[2]))/M_PI*180 < 10)
        {
//        	std::cout<<"Rail angle: "<<acos(mainEV[2])/M_PI*180<<std::endl;
            // Extension in stair direction
            Eigen::Vector2f railLength;
            tempPatch.getRailLength(dirTemp,posTemp,railLength);

			// Compare the front of potential stair rail and the front of the staircase
			//if( railLength[0] > -0.2 )
            if(railLength[0] > -0.2 && (stairRailFlag || railLength[1] < dirTemp.head(2).norm() * (input.stairCount) + 0.25))
            {
	            // Extension in height considering the stair slope
	            Eigen::Vector2f railHeight;
	            tempPatch.getRailHeight(dirTemp,posTemp,railHeight);

	            railHeight[0] += dirTemp[2];
	            railHeight[1] += dirTemp[2];

				//Check the height of the stair rail
				if(railHeight[0] > -0.5 && railHeight[1] < 2)
				{
		            // Extension orthogonal to the stair direction
		            Eigen::Vector2f railWidth;
		            tempPatch.getRailWidth(dirTemp,posTemp,railWidth);

					// Check the width of the stair rail
					if(railWidth[1] - railWidth[0] < 0.2)
					{
						// Check the distance of the stair rail to the right side of stairs
						if( fabs(railWidth[0]) <0.15 )
						{
		//            		if(distance > 0)
		//            		{
		//            			stairCoeff.width += distance;
		//            			stairCoeff.pos.head(2)-=distance*wVec.head(2);
		//            			std::cout<<"Adjusting width Vector left Side"<<std::endl;
		//            		}
							rightCounter += inputRegions.at(regIdx).segmentCloud.size();
							input.stairParts.push_back(inputRegions.at(regIdx));
							input.stairRail.push_back(inputRegions.at(regIdx));
							input.stairRailCloud+=inputRegions.at(regIdx).segmentCloud;
						}
						// Check the distance of the stair rail to the left side of stairs
						if(fabs(railWidth[1] + input.width)<0.15)
						{
		//            		if(distance + stairCoeff.width < 0)
		//            		{
		//            			stairCoeff.width -= stairCoeff.width + distance;
		//            			stairCoeff.pos.head(2)+=distance*wVec.head(2);
		//            			std::cout<<"Adjusting width Vector right Side"<<std::endl;
		//            		}
							leftCounter += inputRegions.at(regIdx).segmentCloud.size();
							input.stairParts.push_back(inputRegions.at(regIdx));
							input.stairRail.push_back(inputRegions.at(regIdx));
							input.stairRailCloud+=inputRegions.at(regIdx).segmentCloud;
						}
					}
				}
			}
        }
    }

    if(leftCounter > 100 && rightCounter > 100)
    {
    	input.anchPoint = 2;
    	input.pos = input.pos - (wVec * 0.5 * input.width);
    }
    else if(leftCounter > rightCounter)
    {
    	input.anchPoint=1;
    	input.pos = input.pos - (wVec * 1 * input.width);
    }
}

void recognition::findStairRailCirc(Stairs& input)
{
	int leftCounter = 0;
	int rightCounter = 0;

	for(int stepIdx = 0; stepIdx < input.stairCount; stepIdx ++) {

		Eigen::Vector3f projNorm;
		Eigen::Matrix2f rotMatrix;
		rotMatrix(0,0) = cosd(input.angleDiff*(stepIdx-0.5));
		if(input.clockwise)
		{
			rotMatrix(0,1) = sind(input.angleDiff*(stepIdx-0.5));
			rotMatrix(1.0) = -sind(input.angleDiff*(stepIdx-0.5));
		}
		else
		{
			rotMatrix(0,1) = -sind(input.angleDiff*(stepIdx-0.5));
			rotMatrix(1.0) = sind(input.angleDiff*(stepIdx-0.5));
		}
		rotMatrix(1,1) = cosd(input.angleDiff*(stepIdx-0.5));
		projNorm.head(2) = input.dir.head(2).transpose() * rotMatrix;
		projNorm[2] = 0;

		Eigen::Vector3f dirTemp = input.dir;
		dirTemp.head(2) = projNorm.head(2);
		Eigen::Vector3f dirVecNorm = dirTemp;
		dirVecNorm.normalize();

		projNorm.normalize();

		Eigen::Vector3f stepRefPos;
		Eigen::Vector2f extensionVec;
		extensionVec << 0,0;
		for(int exIter = 0; exIter < abs(stepIdx); exIter++) {
			int ext = exIter;
			if(stepIdx<0){
				ext+=1;
				ext*=-1;
			}
			rotMatrix(0,0) = cosd(input.angleDiff*ext);
			if(input.clockwise)
			{
				rotMatrix(0,1) = sind(input.angleDiff*ext);
				rotMatrix(1.0) = -sind(input.angleDiff*ext);
			}
			else
			{
				rotMatrix(0,1) = -sind(input.angleDiff*ext);
				rotMatrix(1.0) = sind(input.angleDiff*ext);
			}
			rotMatrix(1,1) = cosd(input.angleDiff*ext);
			if(stepIdx<0)
				extensionVec -= input.dir.head(2).transpose() * rotMatrix;
			else
				extensionVec += input.dir.head(2).transpose() * rotMatrix;
		}
		Eigen::Vector3f posTemp;
		posTemp.head(2) = input.pos.head(2) + extensionVec;
		posTemp[2] = input.pos[2];

//		std::cout<<"Currently checking at stair pos:"<<std::endl;
//		std::cout<<posTemp<<std::endl;

		Eigen::Vector3f horVec;
		horVec = projNorm;

		Eigen::Vector3f zVec;
		zVec<<0,0,1;
		Eigen::Vector3f wVec = horVec.cross(zVec);

		posTemp += wVec * input.widthOff;
		float vertAngle = 90 - (acos(fabs(dirVecNorm[2])))/M_PI*180;

	//    std::cout<<"Stair slope: "<<vertAngle<<std::endl;

		for(int regIdx = 0; regIdx < inputRegions.size(); regIdx++)
		{
			segmentPatch tempPatch = inputRegions.at(regIdx);
			Eigen::Vector3f mainEV = tempPatch.eigen_vectors.col(2);
			// Look planes with same sloop main EV or z-axis is main EV
			if(acos(fabs(mainEV.dot(dirVecNorm)))/M_PI*180 < 45 || acos(fabs(mainEV[2]))/M_PI*180 < 20)
			{
	//        	std::cout<<"Rail angle: "<<acos(mainEV[2])/M_PI*180<<std::endl;
				// Extension in stair direction
				Eigen::Vector2f railLength;
				tempPatch.getRailLength(dirTemp,posTemp,railLength);

				// Compare the front of potential stair rail and the front of the staircase
				//if( railLength[0] > -0.2 )
				if(railLength[0] > -0.2 && (railLength[1] < dirTemp.head(2).norm() + 0.10)) // * (input.stairCount) + 0.25))
				{
					// Extension in height considering the stair slope
					Eigen::Vector2f railHeight;
					tempPatch.getHeight();
//					tempPatch.analyse();
					railHeight[0] = tempPatch.segmentCentroid[2]+tempPatch.height[0]-posTemp[2];
					railHeight[1] = tempPatch.segmentCentroid[2]+tempPatch.height[1]-posTemp[2];

					//Check the height of the stair rail
					if(railHeight[0] > -0.2 && railHeight[1] < 4)
					{
						// Extension orthogonal to the stair direction
						Eigen::Vector2f railWidth;
						tempPatch.getRailWidth(dirTemp,posTemp,railWidth);

						// Check the width of the stair rail
						if(railWidth[1] - railWidth[0] < 0.3)
						{
							// Check the distance of the stair rail to the right side of stairs
							if( fabs(railWidth[0]) <0.1 )
							{
								if(input.clockwise) {
									if(railHeight[0] > stepIdx*dirTemp[2] -0.1 && railHeight[1] < 4) {
										rightCounter += inputRegions.at(regIdx).segmentCloud.size();
										input.stairParts.push_back(inputRegions.at(regIdx));
										input.stairRail.push_back(inputRegions.at(regIdx));
										input.stairRailCloud+=inputRegions.at(regIdx).segmentCloud;
									}
								}
								else {
									rightCounter += inputRegions.at(regIdx).segmentCloud.size();
									input.stairParts.push_back(inputRegions.at(regIdx));
									input.stairRail.push_back(inputRegions.at(regIdx));
									input.stairRailCloud+=inputRegions.at(regIdx).segmentCloud;
								}
							}
							// Check the distance of the stair rail to the left side of stairs
							if(fabs(railWidth[1] + input.width)<0.3)
							{
								if(not(input.clockwise)) {
									if(railHeight[0] > stepIdx*dirTemp[2] -0.1 && railHeight[1] < 4) {
										leftCounter += inputRegions.at(regIdx).segmentCloud.size();
										input.stairParts.push_back(inputRegions.at(regIdx));
										input.stairRail.push_back(inputRegions.at(regIdx));
										input.stairRailCloud+=inputRegions.at(regIdx).segmentCloud;
									}
								}
								else {
									leftCounter += inputRegions.at(regIdx).segmentCloud.size();
									input.stairParts.push_back(inputRegions.at(regIdx));
									input.stairRail.push_back(inputRegions.at(regIdx));
									input.stairRailCloud+=inputRegions.at(regIdx).segmentCloud;
								}
							}
						}
					}
				}
			}
		}
	}

}


void recognition::expandSearch()
{
    startSearchPoint=stairPos;

    int maxLabel = -100;
    int minLabel = 100;
    for(int compIdx=0; compIdx < stairRises.size(); compIdx ++)
    {
    	if(stairRises.at(compIdx).segmentLabel > maxLabel)
    		maxLabel = stairRises.at(compIdx).segmentLabel;
    	if(stairRises.at(compIdx).segmentLabel < minLabel)
    		minLabel = stairRises.at(compIdx).segmentLabel;
    }
    for(int compIdx=0; compIdx < stairTreads.size(); compIdx ++)
    {
    	if(stairTreads.at(compIdx).segmentLabel > maxLabel)
    		maxLabel = stairTreads.at(compIdx).segmentLabel;
    	if(stairTreads.at(compIdx).segmentLabel < minLabel)
    		minLabel = stairTreads.at(compIdx).segmentLabel;
    }
    stepAmount[0] = minLabel;
    stepAmount[1] = maxLabel;
    find();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void recognition::optimizeCoefficients()
{
//	Kostenfunktion erstellen
//
//	Variables:
//	Stair position
//  Stair direction
//  Stair height

	  int n_unknowns = 5;
	  VectorX x (n_unknowns);
	  x.setZero ();

		int minLab = 90;
		int maxLab = -90;

		for(size_t partIdx = 0; partIdx < stairRises.size(); partIdx++)
		{
			if( stairRises.at(partIdx).segmentLabel<minLab)
				minLab =  stairRises.at(partIdx).segmentLabel;
			if( stairRises.at(partIdx).segmentLabel>maxLab)
				maxLab =  stairRises.at(partIdx).segmentLabel;
		}
		for(size_t partIdx = 0; partIdx <  stairTreads.size(); partIdx++)
		{
			if( stairTreads.at(partIdx).segmentLabel<minLab)
				minLab =  stairTreads.at(partIdx).segmentLabel;
			if( stairTreads.at(partIdx).segmentLabel>maxLab)
				maxLab =  stairTreads.at(partIdx).segmentLabel;
		}


		int stepSize = maxLab - minLab + 1;

		int dataPoints = stepSize*7 + stepSize*3;
//	  int dataPoints = stairRises.size()*7 + stairTreads.size()*3;

	  OptimizationFunctor functor (dataPoints, this);
	  Eigen::NumericalDiff<OptimizationFunctor> num_diff (functor);
	  //Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, double> lm (num_diff);
	  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, float> lm (num_diff);

	  int info = lm.minimize (x);

	  // Compute the norm of the residuals
	  Eigen::Vector3f changeVec;
	  changeVec.head(2) = distVec.head(2) * x[0];
	  changeVec[2] = x[1];

	  Eigen::Matrix<float,5,1> blub;
	  blub << 0,0,0,0,0;
//	  std::cout<<"Error: "<<(computeDistance(blub,stairRises,stairTreads)).norm()/dataPoints<<std::endl;

	  stairPos=stairPos + changeVec;
	  distVec=distVec + x.tail(3);
}



VectorX recognition::computeDistanceNew(Eigen::Matrix<float,5,1> estimateVector, regions nstairRises, regions nstairTreads) const
{
	int minLab = 90;
	int maxLab = -90;

	for(int partIdx = 0; partIdx < nstairRises.size(); partIdx++)
	{
		if( nstairRises.at(partIdx).segmentLabel<minLab)
			minLab =  nstairRises.at(partIdx).segmentLabel;
		if( nstairRises.at(partIdx).segmentLabel>maxLab)
			maxLab =  nstairRises.at(partIdx).segmentLabel;
	}
	for(int partIdx = 0; partIdx <  nstairTreads.size(); partIdx++)
	{
		if( nstairTreads.at(partIdx).segmentLabel<minLab)
			minLab =  nstairTreads.at(partIdx).segmentLabel;
		if( nstairTreads.at(partIdx).segmentLabel>maxLab)
			maxLab =  nstairTreads.at(partIdx).segmentLabel;
	}


	int stepSize = maxLab - minLab + 1;

	int srcals=7;
	int stcalcs=3;

	VectorX errorVec (stepSize*srcals + stepSize*stcalcs);
	errorVec.setZero();

	Eigen::Vector3f estimateDir;
	estimateDir = estimateVector.tail(3);
	estimateDir += distVec;
	float stairWidth = estimateDir.head(2).norm();

	Eigen::Vector3f estimateNormDir;
	estimateNormDir=estimateDir;
	estimateNormDir[2]=0;
	estimateNormDir.normalize();

	Eigen::Vector3f estStairPos;
	estStairPos = startSearchPoint;
	estStairPos.head(2) += (estimateVector[0]*(estimateDir.head(2)));
	estStairPos[2] += estimateVector[1];

	int currStep=0;
	for(int stepIdx = minLab; stepIdx <= maxLab; stepIdx ++)
	{
		regions currRisers;
		segmentPatch currRiserPatch;
		for(int stairRiseIdx = 0; stairRiseIdx <  nstairRises.size(); stairRiseIdx++)
		{
			if( nstairRises.at(stairRiseIdx).segmentLabel == stepIdx)
			{
				currRiserPatch.segmentCloud+= nstairRises.at(stairRiseIdx).segmentCloud;
				currRisers.push_back( nstairRises.at(stairRiseIdx));
			}
		}
		regions currTreads;
		segmentPatch currTreadPatch;
		for(int stairTreadIdx = 0; stairTreadIdx <  nstairTreads.size(); stairTreadIdx++)
		{
			if( nstairTreads.at(stairTreadIdx).segmentLabel == stepIdx)
			{
				currTreadPatch.segmentCloud+= nstairTreads.at(stairTreadIdx).segmentCloud;
				currTreads.push_back( nstairTreads.at(stairTreadIdx));
			}
		}

		// Check horizontal distance //
		if(currRisers.size() >0)
		{
			float posErr;
			float coeffErr;
			currRiserPatch.analyse();

			posErr = fabs(((estStairPos.head(2) + stepIdx * estimateDir.head(2)) - currRiserPatch.segmentCentroid.head(2)).dot(estimateNormDir.head(2)));

			if(stepIdx !=0)
				coeffErr = posErr / abs(stepIdx);
			else
			{
				coeffErr = 0;
			}
			errorVec[currStep*srcals]=posErr*distVec[0];
			errorVec[currStep*srcals+1]=posErr*distVec[1];
			errorVec[currStep*srcals+2]=coeffErr*distVec[0];
			errorVec[currStep*srcals+3]=coeffErr*distVec[1];

			Eigen::Vector3f segCoeffNorm = currRiserPatch.segmentCoefficient.head(3);

			segCoeffNorm[2] = 0;
			segCoeffNorm.normalize();

			float angleMismatch = fabs(segCoeffNorm.head(2).dot(estimateNormDir.head(2)));
			if(angleMismatch > 0.99999)
				angleMismatch = 0.99999;
			errorVec[currStep*srcals+4]=acos(angleMismatch);
			if(errorVec[currStep*srcals+4] != errorVec[currStep*srcals+4])
				std::cout<<"ROUNDING ERROR"<<std::endl;
			float updateCut = 0;
			float updateExtend = 0;

			float vDist = currRiserPatch.segmentCentroid[2] - estStairPos[2];

			currRiserPatch.getHeight();
			posErr = fabs((estStairPos[2] + stepIdx * estimateDir[2]) - (currRiserPatch.segmentCentroid[2] + currRiserPatch.height[1]));
			if(stepIdx !=0)
				coeffErr = posErr / abs(stepIdx);
			else
			{
				coeffErr = 0;
			}
			errorVec[2] += posErr;
			errorVec[currStep*srcals+5]=posErr*distVec[2];
			errorVec[currStep*srcals+6]=coeffErr*distVec[2];
		}
		else
		{
			errorVec[currStep*srcals]=0;
			errorVec[currStep*srcals+1]=0;
			errorVec[currStep*srcals+2]=0;
			errorVec[currStep*srcals+3]=0;
			errorVec[currStep*srcals+4]=0;
			errorVec[currStep*srcals+5]=0;
			errorVec[currStep*srcals+6]=0;
		}

		if(currTreads.size() >0)
		{
			// Check vertical distance //
			segmentPatch tempPatch = currTreadPatch;
			tempPatch.segmentLabel=stepIdx;
			tempPatch.analyse();
			float posErr;
			float coeffErr;
			posErr = fabs((estStairPos[2] + tempPatch.segmentLabel * estimateDir[2]) - tempPatch.segmentCentroid[2]);
	//		cout<<posErr<<"   "<<estimateVector[2]<<"  "<<stairPos[2]<<" - "<<tempPatch.segmentCentroid[2]<<"   "<<tempPatch.segmentLabel * (distVec[2]+estimateDir[2]) <<endl;;
			if(tempPatch.segmentLabel !=0)
				coeffErr = posErr / abs(tempPatch.segmentLabel);
			else coeffErr = 0;
			errorVec[stepSize*srcals+currStep*stcalcs]=posErr;
			errorVec[stepSize*srcals+currStep*stcalcs+1]=coeffErr;

			float hDist = (tempPatch.segmentCentroid.head(2) - estStairPos.head(2)).dot(estimateNormDir.head(2)); // Horizontal distance to center

			tempPatch.getTreadDepth(estimateNormDir);

			float updateCut = 0;
			float updateExtend = 0;

			if(tempPatch.segmentLabel >= 0)
			{
				if( hDist + tempPatch.depth[1] > (tempPatch.segmentLabel+1)*stairWidth ) // Vertical distance to stair top
				{
					updateExtend =  hDist + tempPatch.depth[1] - (tempPatch.segmentLabel+1)*stairWidth;
				}
				// Check if stair height too small for ascending or too large for descending
				if( hDist + tempPatch.depth[0] < (tempPatch.segmentLabel)*stairWidth ) // Vertical distance to stair bottom
				{
					updateCut = hDist + tempPatch.depth[0] - (tempPatch.segmentLabel)*stairWidth;
				}
			}
			else
			{
				if( hDist + tempPatch.depth[1] > (tempPatch.segmentLabel+1)*stairWidth ) // Vertical distance to stair top
				{
					updateCut = (tempPatch.segmentLabel+1)*stairWidth -  (hDist + tempPatch.depth[1]);
				}
				// Check if stair height too small for ascending or too large for descending
				if( hDist + tempPatch.depth[0] < (tempPatch.segmentLabel)*stairWidth ) // Vertical distance to stair bottom
				{
					updateExtend = (tempPatch.segmentLabel)*stairWidth - (hDist + tempPatch.depth[0]) ;
				}
			}
			if(false && nstairRises.size() < 3)
				errorVec[stepSize*srcals+currStep*stcalcs+2]=fabs(updateCut)+fabs(updateExtend);
			else
				errorVec[stepSize*srcals+currStep*stcalcs+2]=0;
		}
		else
		{
			errorVec[stepSize*srcals+currStep*stcalcs]=0;
			errorVec[stepSize*srcals+currStep*stcalcs+1]=0;
			errorVec[stepSize*srcals+currStep*stcalcs+2]=0;
		}
		currStep++;
	}
//	std::cout<<"Printing error vec"<<std::endl;
//	std::cout<<errorVec<<std::endl;
	return errorVec;
}


int recognition::OptimizationFunctor::operator () (
    const VectorX &x, VectorX &fvec) const
{
	Eigen::Matrix<float,5,1> estVec;
	estVec.head(5) = x.head(5);

	fvec = estimator_->computeDistanceNew(estVec, estimator_->stairRises, estimator_->stairTreads);
	return (0);
}


