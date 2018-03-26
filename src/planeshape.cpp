#include <stairs/planeshape.h>

planeshape::planeshape()
{
	// Angle difference allowed to horizontal (treads) or vertical (risers) plane
	angleMargin = 30.0;
	// Width requirement
	widthReq << 0.0,10.0;
	treadDepth << 0,0.50;
	riserHeight << 0,0.24;
}

void planeshape::filterSc(regions& stairTreads, regions& stairRisers)
{
	stairTreads.clear();
	stairRisers.clear();

    for(int regCounter=0; regCounter < segments.size(); regCounter++)
    {
        float z_dir_main_ev = 90 - acos(fabs(segments.at(regCounter).eigen_vectors(2,2)))/M_PI*180;
        float z_dir = acos(fabs(segments.at(regCounter).eigen_vectors(2,0)))/M_PI*180;
        if(z_dir < angleMargin && fabs(z_dir_main_ev) < angleMargin)
        {
            Eigen::Vector3f dimensions = segments.at(regCounter).middleExtensions;
            if(dimensions[0] > widthReq[0] && dimensions[0] < widthReq[1])
            {
                if(dimensions[1] > treadDepth[0] && dimensions[1] < treadDepth[1])
                {
                	stairTreads.push_back(segments.at(regCounter));
                }
            }
        }
        if(fabs(z_dir-90) < angleMargin && fabs(z_dir_main_ev) < angleMargin)
        {
            Eigen::Vector3f dimensions = segments.at(regCounter).middleExtensions;
            if(dimensions[0] > widthReq[0] && dimensions[0] < widthReq[1])
            {
                if(dimensions[1] > riserHeight[0] && dimensions[1] < riserHeight[1])
                {
                    segments.at(regCounter).getMaxHeight();
                    stairRisers.push_back(segments.at(regCounter));
                }
            }

        }
    }
}
