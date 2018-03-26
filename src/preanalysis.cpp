#include <stairs/preanalysis.h>

Preanalysis::Preanalysis()
{
	// Set if downsample active
	dsFlag = true;
	// Set downsample resolution
	dsResolution = 0.01;



	// Normal estimation - find N neareast neighbors (:=0) - find points within distance (:=1)
	neNeighMethod = 0;
	neSearchNeighbours = 24;
	neSearchRadius = 0.2;

	// Ghost point filter active?
	gpFlag = true;
	// Ghost point filter angle
	gpAngle = 5.0;

	// Point normal filter active?
	pfActive = false;
	// Point normal filter angle
	pfAngle = 20.0;

	// Floor seperation active?
	fsActive = false;
    // Floor seperation angle
    fsAngle = 30.0;;
    // Floor seperation distance
    fsRange = 0.05;

    // Set the position of the LIDAR (required for floor separation)
    rob_x=0.00;
    rob_y=0.00;
    rob_z=0.00;

    // Rotate pointcloud around z-axis
    robAngle = 0;

	// Downsample method - Standard: flase - Experimental version: true
	dsMethod =false;
    // Process ghost point filter and floor separation in separate steps
	neMethod = 0;

    inputCloud.reset (new PointCloudT);
    normal_cloud.reset (new NormalCloud);
}


void Preanalysis::run(PointCloudT::Ptr& input, NormalCloud::Ptr& normal, PointCloudC& colMap, PointCloudT& floorPoints)
{
	pc.reset (new PointCloudT);
    inputCloud=input;

    Eigen::Matrix4f transformCloud = Eigen::Matrix4f::Identity();

    transformCloud(0,3) = 0;//-0.2;
    pcl::transformPointCloud (*inputCloud, *inputCloud, transformCloud);

    transformCloud(0,3) = 0;

    transformCloud(0,0) = cos(robAngle);
    transformCloud(1,1) = cos(robAngle);
    transformCloud(0,1) = -sin(robAngle);
    transformCloud(1,0) = sin(robAngle);
    pcl::transformPointCloud (*inputCloud, *inputCloud, transformCloud);


    if(dsFlag)
        downsample();
    else
        pc=inputCloud;
    double ne_start = pcl::getTime();

	normalEstimation();

    double ne_end = pcl::getTime();
    neTime = ne_end - ne_start;

    if(neMethod != 0)
    {
		if(gpFlag)
			ghostPointFilter();

		if(fsActive)
			floorExtraction();
    }

    normal=normal_cloud;
    input=pc;

    floorPoints=floorPC;

    for (size_t i_point = 0; i_point < pc->size (); i_point++)
    {
        pcl::PointXYZRGB point;
        point.x=pc->at(i_point).x;
        point.y=pc->at(i_point).y;
        point.z=pc->at(i_point).z;
        point.r = abs(round(255*cbrt(normal->at(i_point).normal_x)));
        point.g = abs(round(255*cbrt(normal->at(i_point).normal_y)));
        point.b = abs(round(255*cbrt(normal->at(i_point).normal_z)));
        colMap.push_back(point);
    }
}

void Preanalysis::limitPC()
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (inputCloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-10.23, 10.23);
    pass.filter (*inputCloud);
    pass.setInputCloud (inputCloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-10.23, 10.23);
    pass.filter (*inputCloud);
    pass.setInputCloud (inputCloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-10.23, 10.23);
    pass.filter (*inputCloud);
}

void Preanalysis::downsample()
{
    float octreeResolution = dsResolution;
    pcl::octree::OctreePointCloud<pcl::PointXYZ> octree (octreeResolution);
    octree.setInputCloud (inputCloud);
    octree.addPointsFromInputCloud ();

    pcl::octree::OctreePointCloud<pcl::PointXYZ>::LeafNodeIterator itL (&octree);
    if(not(dsMethod))
    {
		do
		{
			std::vector<int> indicesVec;
			LeafContainer lf;
			lf=itL.getLeafContainer();
			lf.getPointIndices(indicesVec);
			Eigen::Vector3f centP;
			PointT centerPoint;
			centP<<0,0,0;
			for(size_t voxelPoint=0;voxelPoint<indicesVec.size();voxelPoint++)
			{
				centP[0]+=inputCloud->at(indicesVec[voxelPoint]).x;
				centP[1]+=inputCloud->at(indicesVec[voxelPoint]).y;
				centP[2]+=inputCloud->at(indicesVec[voxelPoint]).z;
			}
			centP/=indicesVec.size();
			centerPoint.x=centP[0];
			centerPoint.y=centP[1];
			centerPoint.z=centP[2];
			pc->push_back(centerPoint);

		}while(*++itL);
    }
    else
    {
		do
		{
			std::vector<int> indicesVec;
			LeafContainer lf;
			lf=itL.getLeafContainer();
			lf.getPointIndices(indicesVec);

			pc->push_back(inputCloud->at(indicesVec[0]));

		}while(*++itL);
    }
}

void Preanalysis::normalEstimation()
{
    pcl::NormalEstimationOMP<PointT, Normal> ne;
    ne.setInputCloud (pc);
    pcl::search::KdTree<PointT>::Ptr tree_n (new pcl::search::KdTree<PointT>());
    ne.setSearchMethod (tree_n);


    ne.setGpActive(gpFlag);
    gpAngle = gpAngle / 180 * M_PI;
    gpAngle = sin(gpAngle);
    ne.setGpAngle(gpAngle);

    ne.setFsActive(fsActive);
    ne.setFsAngle(fsAngle);
    ne.setFsRange(fsRange);

    ne.setPfActive(pfActive);
    ne.setPfAngle(pfAngle);

    ne.setNumberOfThreads(1); // Inconsistent to suddenly use more cores //

//    neSearchNeighbours=8;

    if(neNeighMethod == 0)
    {
        ne.setKSearch(neSearchNeighbours);
    }
    else if(neNeighMethod == 1)
    {
        ne.setRadiusSearch (neSearchRadius);
    }

	ne.compute (*normal_cloud);



	// Floor separation //
//	std::vector<int> floorPoints;
	boost::shared_ptr <std::vector<int> > flIndicesPtr (new std::vector<int>);
	boost::shared_ptr <std::vector<int> > gpIndicesPtr (new std::vector<int>);
	boost::shared_ptr <std::vector<int> > npIndicesPtr (new std::vector<int>);

//	std::vector<int>::pointer floorPtr;
//	floorPtr = &floorPoints;
	ne.getFloorIndices(*flIndicesPtr);
	ne.getGhostIndices(*gpIndicesPtr);
	ne.getNormalIndices(*npIndicesPtr);

	boost::shared_ptr <std::vector<int> > wholeIndicesPtr (new std::vector<int>);

	wholeIndicesPtr->reserve( flIndicesPtr->size() + gpIndicesPtr->size() + npIndicesPtr->size()); // preallocate memory
	wholeIndicesPtr->insert( wholeIndicesPtr->end(), flIndicesPtr->begin(), flIndicesPtr->end() );
	wholeIndicesPtr->insert( wholeIndicesPtr->end(), gpIndicesPtr->begin(), gpIndicesPtr->end() );
	wholeIndicesPtr->insert( wholeIndicesPtr->end(), npIndicesPtr->begin(), npIndicesPtr->end() );


	PointCloudT floorlessPC;
	NormalCloud floorlessNormal;

	pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (pc);
    extract.setIndices (flIndicesPtr);
    extract.setNegative (false);
    extract.filter (floorPC);
    extract.setIndices(wholeIndicesPtr);
    extract.setNegative (true);
    extract.filter (*pc);

	pcl::ExtractIndices<Normal> extractNormal;
	extractNormal.setInputCloud (normal_cloud);
	extractNormal.setIndices (flIndicesPtr);
	extractNormal.setNegative (false);
	extractNormal.filter (floorNormal);
	extractNormal.setIndices(wholeIndicesPtr);
	extractNormal.setNegative (true);
	extractNormal.filter (*normal_cloud);


}

void Preanalysis::ghostPointFilter()
{
    PointCloudT purgedPC;
    PointCloudT delPC;
    NormalCloud::Ptr purgedNormals (new NormalCloud);
    for(size_t pointIter = 0; pointIter < pc->size(); pointIter++)
    {
        Eigen::Vector3f pointPosition;
        Eigen::Vector3f pointNormal;
        pointPosition[0] = pc->at(pointIter).x-rob_x;
        pointPosition[1] = pc->at(pointIter).y-rob_y;
        pointPosition[2] = pc->at(pointIter).z-rob_z;
        pointPosition.normalize();
        pointNormal[0] = normal_cloud->at(pointIter).normal_x;
        pointNormal[1] = normal_cloud->at(pointIter).normal_y;
        pointNormal[2] = normal_cloud->at(pointIter).normal_z;
        if(asin(fabs(pointPosition.dot(pointNormal)))/M_PI*180 > gpAngle)
        {
            purgedPC.push_back(pc->at(pointIter));
            purgedNormals->push_back(normal_cloud->at(pointIter));
        }
        else
        {
            delPC.push_back(pc->at(pointIter));
        }
    }
    *pc = purgedPC;
    normal_cloud = purgedNormals;
}

void Preanalysis::floorExtraction()
{
   float z_low = -fsRange/2;
   float z_high = -z_low;

   PointCloudT floorlessPC;
   NormalCloud floorlessNormal;

   Indices floorIndices;
   Indices floorlessIndices;

   for(size_t filtCounter=0;filtCounter < pc->size();filtCounter++)
   {
       if(pc->at(filtCounter).z < z_high && pc->at(filtCounter).z > z_low)
       {
           floorPC.push_back(pc->at(filtCounter));
           floorNormal.push_back(normal_cloud->at(filtCounter));
           floorIndices.push_back(filtCounter);
       }
       else
       {
           floorlessPC.push_back(pc->at(filtCounter));
           floorlessNormal.push_back(normal_cloud->at(filtCounter));
           floorlessIndices.push_back(filtCounter);
       }
   }

   Eigen::Vector3f zAxis;
   zAxis << 0,0,1;
   Eigen::Vector3f currNorm;

   for(size_t pointCounter=0;pointCounter<floorIndices.size();pointCounter++)
   {
       currNorm[0]=normal_cloud->at(floorIndices.at(pointCounter)).normal_x;
       currNorm[1]=normal_cloud->at(floorIndices.at(pointCounter)).normal_y;
       currNorm[2]=normal_cloud->at(floorIndices.at(pointCounter)).normal_z;

       if(acos(fabs(zAxis.dot(currNorm)))/M_PI*180 > fsAngle)
       {
           floorlessPC.push_back(floorPC.at(pointCounter));
           floorlessNormal.push_back(floorNormal.at(pointCounter));
           floorlessIndices.push_back(floorIndices.at(pointCounter));

           floorPC.erase(floorPC.begin()+pointCounter,floorPC.begin()+pointCounter+1);
           floorNormal.erase(floorNormal.begin()+pointCounter,floorNormal.begin()+pointCounter+1);
           floorIndices.erase(floorIndices.begin()+pointCounter,floorIndices.begin()+pointCounter+1);
           pointCounter--;
           if(floorPC.size() == 0)
           {
               std::cout<<"NO FLOOR DETECTED"<<std::endl;
           }
       }
   }
   *pc = floorlessPC;
   *normal_cloud = floorlessNormal;

}
