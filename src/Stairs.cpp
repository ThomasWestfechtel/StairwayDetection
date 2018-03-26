/*
 * Stairs.cpp
 *
 *  Created on: Oct 26, 2015
 *      Author: tom
 */

#include "stairs/Stairs.h"

//using namespace std;

Stairs::Stairs() {
	// TODO Auto-generated constructor stub
	pos << 0,0,0;
	dir << 0,0,0;
	width = 0;
	anchPoint = 0;
	isCircular = false;
	angleDiff = 0;
	clockwise = true;
	widthOff = 0;
}

//Stairs::~Stairs() {
//	// TODO Auto-generated destructor stub
//}

void Stairs::getColoredParts(PointCloudC& output)
{
	PointTC colPoint;
	for(int pointIdx = 0; pointIdx < stairRiseCloud.size(); pointIdx++)
	{
		colPoint.x = stairRiseCloud[pointIdx].x;
		colPoint.y = stairRiseCloud[pointIdx].y;
		colPoint.z = stairRiseCloud[pointIdx].z;
		colPoint.r=255;
		colPoint.g=0;
		colPoint.b=0;
		output.push_back(colPoint);
	}

	for(int pointIdx = 0; pointIdx < stairTreadCloud.size(); pointIdx++)
	{
		colPoint.x = stairTreadCloud[pointIdx].x;
		colPoint.y = stairTreadCloud[pointIdx].y;
		colPoint.z = stairTreadCloud[pointIdx].z;
		colPoint.r=0;
		colPoint.g=0;
		colPoint.b=255;
		output.push_back(colPoint);
	}

	for(int pointIdx = 0; pointIdx < stairRailCloud.size(); pointIdx++)
	{
		colPoint.x = stairRailCloud[pointIdx].x;
		colPoint.y = stairRailCloud[pointIdx].y;
		colPoint.z = stairRailCloud[pointIdx].z;
		colPoint.r=0;
		colPoint.g=255;
		colPoint.b=0;
		output.push_back(colPoint);
	}
}

std::ostream& operator<<(std::ostream& os, Stairs& sc) {
	os << "Stair Position: "<<sc.pos[0]<<"   "<<sc.pos[1]<<"   "<<sc.pos[2]<<std::endl;
	os << "Stair Direction: "<<sc.dir[0]<<"   "<<sc.dir[1]<<"   "<<sc.dir[2]<<std::endl;
	os << "Stair Width: "<<sc.width<<std::endl;
	os << "Stair Anchor: "<<sc.anchPoint;
	return os;
}

