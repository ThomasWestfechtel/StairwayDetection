/*
 * StairVector.cpp
 *
 *  Created on: Oct 26, 2015
 *      Author: tom
 */

#include "stairs/StairVector.h"

StairVector::StairVector() {
	// TODO Auto-generated constructor stub

}

//StairVector::~StairVector() {
//	// TODO Auto-generated destructor stub
//}

regions StairVector::getAllRegions()
{
	regions output;
	for(size_t vecPos = 0; vecPos < stairVector.size(); vecPos++)
	{
		for(size_t regionPos = 0; regionPos < stairVector.at(vecPos).size(); regionPos++)
		{
			output.push_back(stairVector.at(vecPos).at(regionPos));
		}
	}
	return output;
}

PointCloudC StairVector::getColoredCloud(int pos)
{
	PointCloudC output;
	output += stairVector.at(pos).stairParts.getColoredCloud();
//	for(size_t vecPos = 0; vecPos < 1; vecPos++)
//	{
//		output += stairVector.at(vecPos).stairParts.getColoredCloud();
//	}
	return output;
}

PointCloudC StairVector::getColoredParts(int pos)
{
	PointCloudC output;
	stairVector.at(pos).getColoredParts(output);
	return output;
}

void StairVector::sort()
{
	std::sort(stairVector.begin(), stairVector.end());
}
