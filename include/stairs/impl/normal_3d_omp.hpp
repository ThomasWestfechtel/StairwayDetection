/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_FEATURES_IMPL_NORMAL_3D_OMP_H_
#define PCL_FEATURES_IMPL_NORMAL_3D_OMP_H_

#include <stairs/normal_3d_omp.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::NormalEstimationOMP<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);
  std::vector<bool> floorPositions (input_->size(),false);
  std::vector<bool> ghostPointsPos (input_->size(),false);
  std::vector<bool> normalFilterPos (input_->size(),false);
  Eigen::Vector3f rob_pos;
  rob_pos << 0.2 , 0, 0.35;

  output.is_dense = true;

  // Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
  if (input_->is_dense)
  {
	  if(gpActive && fsActive && pfActive)
	  {
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
    // Iterating over the entire index vector
    for (int idx = 0; idx < static_cast<int> (indices_->size ()); ++idx)
    {
      if (this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
      {
        output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();

        output.is_dense = false;
        continue;
      }

      Eigen::Vector4f n;
      pcl::computePointNormal<PointInT> (*surface_, nn_indices,
                                         n,
                                         output.points[idx].curvature);
                          
      output.points[idx].normal_x = n[0];
      output.points[idx].normal_y = n[1];
      output.points[idx].normal_z = n[2];
  
      flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                  output.points[idx].normal[0], 
                                  output.points[idx].normal[1], 
                                  output.points[idx].normal[2]);

      //Floor separation
      if(fabs(input_->points[idx].z) < fsRange)
      {
    	  if(fabs(output.points[idx].normal_z) > fsAngle)
    	  {
    		  floorPositions[idx] = true;
    		  continue;
    	  }
      }

      //Normal filtration
      if(fabs(n[2]) < pfAngleHigh && fabs(n[2]) > pfAngleLow)
      {
    	  normalFilterPos[idx]=true;
    	  continue;
      }

      //Ghost point elimination
      Eigen::Vector3f pointPos;
      pointPos << input_->points[idx].x, input_->points[idx].y, input_->points[idx].z;
      pointPos -=rob_pos;
      pointPos.normalize();
      if( fabs(pointPos.dot(n.head(3))) < gpAngle )
      {
    	  ghostPointsPos[idx]=true;
    	  continue;
      }

    }
  for(size_t copyIdx = 0; copyIdx < floorPositions.size();copyIdx++)
  {
	  if(floorPositions[copyIdx])
		  floorIndices.push_back(copyIdx);
	  if(ghostPointsPos[copyIdx])
		  ghostPointIndices.push_back(copyIdx);
	  if(normalFilterPos[copyIdx])
		  normalFilterIndices.push_back(copyIdx);
  }
  }
	  else  if(gpActive && fsActive && not(pfActive))
	  {
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
    // Iterating over the entire index vector
    for (int idx = 0; idx < static_cast<int> (indices_->size ()); ++idx)
    {
      if (this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
      {
        output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();

        output.is_dense = false;
        continue;
      }

      Eigen::Vector4f n;
      pcl::computePointNormal<PointInT> (*surface_, nn_indices,
                                         n,
                                         output.points[idx].curvature);

      output.points[idx].normal_x = n[0];
      output.points[idx].normal_y = n[1];
      output.points[idx].normal_z = n[2];

      flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                  output.points[idx].normal[0],
                                  output.points[idx].normal[1],
                                  output.points[idx].normal[2]);

      //Floor separation
      if(fabs(input_->points[idx].z) < fsRange)
      {
    	  if(fabs(output.points[idx].normal_z) > fsAngle)
    	  {
    		  floorPositions[idx] = true;
    		  continue;
    	  }
      }

      //Ghost point elimination
      Eigen::Vector3f pointPos;
      pointPos << input_->points[idx].x, input_->points[idx].y, input_->points[idx].z;
      pointPos -=rob_pos;
      pointPos.normalize();
      if( fabs(pointPos.dot(n.head(3))) < gpAngle )
      {
    	  ghostPointsPos[idx]=true;
    	  continue;
      }
    }
  for(size_t copyIdx = 0; copyIdx < floorPositions.size();copyIdx++)
  {
	  if(floorPositions[copyIdx])
		  floorIndices.push_back(copyIdx);
	  if(ghostPointsPos[copyIdx])
		  ghostPointIndices.push_back(copyIdx);
  }
  }
	  else  if(gpActive && not(fsActive) && pfActive)
	  {
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
    // Iterating over the entire index vector
    for (int idx = 0; idx < static_cast<int> (indices_->size ()); ++idx)
    {
      if (this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
      {
        output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();

        output.is_dense = false;
        continue;
      }

      Eigen::Vector4f n;
      pcl::computePointNormal<PointInT> (*surface_, nn_indices,
                                         n,
                                         output.points[idx].curvature);

      output.points[idx].normal_x = n[0];
      output.points[idx].normal_y = n[1];
      output.points[idx].normal_z = n[2];

      flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                  output.points[idx].normal[0],
                                  output.points[idx].normal[1],
                                  output.points[idx].normal[2]);

      //Normal filtration
      if(fabs(n[2]) < pfAngleHigh && fabs(n[2]) > pfAngleLow)
      {
    	  normalFilterPos[idx]=true;
    	  continue;
      }

      //Ghost point elimination
      Eigen::Vector3f pointPos;
      pointPos << input_->points[idx].x, input_->points[idx].y, input_->points[idx].z;
      pointPos -=rob_pos;
      pointPos.normalize();
      if( fabs(pointPos.dot(n.head(3))) < gpAngle )
      {
    	  ghostPointsPos[idx]=true;
    	  continue;
      }
    }
  for(size_t copyIdx = 0; copyIdx < floorPositions.size();copyIdx++)
  {
	  if(ghostPointsPos[copyIdx])
		  ghostPointIndices.push_back(copyIdx);
	  if(normalFilterPos[copyIdx])
		  normalFilterIndices.push_back(copyIdx);
  }
  }
	  else  if(not(gpActive) && fsActive && pfActive)
	  {
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
    // Iterating over the entire index vector
    for (int idx = 0; idx < static_cast<int> (indices_->size ()); ++idx)
    {
      if (this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
      {
        output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();

        output.is_dense = false;
        continue;
      }

      Eigen::Vector4f n;
      pcl::computePointNormal<PointInT> (*surface_, nn_indices,
                                         n,
                                         output.points[idx].curvature);

      output.points[idx].normal_x = n[0];
      output.points[idx].normal_y = n[1];
      output.points[idx].normal_z = n[2];

      flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                  output.points[idx].normal[0],
                                  output.points[idx].normal[1],
                                  output.points[idx].normal[2]);

      //Floor separation
      if(fabs(input_->points[idx].z) < fsRange)
      {
    	  if(fabs(output.points[idx].normal_z) > fsAngle)
    	  {
    		  floorPositions[idx] = true;
    		  continue;
    	  }
      }

      //Normal filtration
      if(fabs(n[2]) < pfAngleHigh && fabs(n[2]) > pfAngleLow)
      {
    	  normalFilterPos[idx]=true;
    	  continue;
      }
    }
  for(size_t copyIdx = 0; copyIdx < floorPositions.size();copyIdx++)
  {
	  if(floorPositions[copyIdx])
		  floorIndices.push_back(copyIdx);
	  if(normalFilterPos[copyIdx])
		  normalFilterIndices.push_back(copyIdx);
  }
  }
	  else  if(not(gpActive) && not(fsActive) && pfActive)
	  {
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
    // Iterating over the entire index vector
    for (int idx = 0; idx < static_cast<int> (indices_->size ()); ++idx)
    {
      if (this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
      {
        output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();

        output.is_dense = false;
        continue;
      }

      Eigen::Vector4f n;
      pcl::computePointNormal<PointInT> (*surface_, nn_indices,
                                         n,
                                         output.points[idx].curvature);

      output.points[idx].normal_x = n[0];
      output.points[idx].normal_y = n[1];
      output.points[idx].normal_z = n[2];

      flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                  output.points[idx].normal[0],
                                  output.points[idx].normal[1],
                                  output.points[idx].normal[2]);

      //Normal filtration
      if(fabs(n[2]) < pfAngleHigh && fabs(n[2]) > pfAngleLow)
      {
    	  normalFilterPos[idx]=true;
    	  continue;
      }

    }
  for(size_t copyIdx = 0; copyIdx < floorPositions.size();copyIdx++)
  {
	  if(normalFilterPos[copyIdx])
		  normalFilterIndices.push_back(copyIdx);
  }
  }
	  else  if(not(gpActive) && fsActive && not(pfActive))
	  {
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
    // Iterating over the entire index vector
    for (int idx = 0; idx < static_cast<int> (indices_->size ()); ++idx)
    {
      if (this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
      {
        output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();

        output.is_dense = false;
        continue;
      }

      Eigen::Vector4f n;
      pcl::computePointNormal<PointInT> (*surface_, nn_indices,
                                         n,
                                         output.points[idx].curvature);

      output.points[idx].normal_x = n[0];
      output.points[idx].normal_y = n[1];
      output.points[idx].normal_z = n[2];

      flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                  output.points[idx].normal[0],
                                  output.points[idx].normal[1],
                                  output.points[idx].normal[2]);

      //Floor separation
      if(fabs(input_->points[idx].z) < fsRange)
      {
    	  if(fabs(output.points[idx].normal_z) > fsAngle)
    	  {
    		  floorPositions[idx] = true;
    		  continue;
    	  }
      }
    }
  for(size_t copyIdx = 0; copyIdx < floorPositions.size();copyIdx++)
  {
	  if(floorPositions[copyIdx])
		  floorIndices.push_back(copyIdx);
  }
  }
	  else  if(gpActive && not(fsActive) && not(pfActive))
	  {
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
    // Iterating over the entire index vector
    for (int idx = 0; idx < static_cast<int> (indices_->size ()); ++idx)
    {
      if (this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
      {
        output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();

        output.is_dense = false;
        continue;
      }

      Eigen::Vector4f n;
      pcl::computePointNormal<PointInT> (*surface_, nn_indices,
                                         n,
                                         output.points[idx].curvature);

      output.points[idx].normal_x = n[0];
      output.points[idx].normal_y = n[1];
      output.points[idx].normal_z = n[2];

      flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                  output.points[idx].normal[0],
                                  output.points[idx].normal[1],
                                  output.points[idx].normal[2]);

      //Ghost point elimination
      Eigen::Vector3f pointPos;
      pointPos << input_->points[idx].x, input_->points[idx].y, input_->points[idx].z;
      pointPos -=rob_pos;
      pointPos.normalize();
      if( fabs(pointPos.dot(n.head(3))) < gpAngle )
      {
    	  ghostPointsPos[idx]=true;
    	  continue;
      }

    }
  for(size_t copyIdx = 0; copyIdx < floorPositions.size();copyIdx++)
  {
	  if(ghostPointsPos[copyIdx])
		  ghostPointIndices.push_back(copyIdx);
  }
  }
	  else {
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
    // Iterating over the entire index vector
    for (int idx = 0; idx < static_cast<int> (indices_->size ()); ++idx)
    {
      if (this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
      {
        output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();

        output.is_dense = false;
        continue;
      }

      Eigen::Vector4f n;
      pcl::computePointNormal<PointInT> (*surface_, nn_indices,
                                         n,
                                         output.points[idx].curvature);

      output.points[idx].normal_x = n[0];
      output.points[idx].normal_y = n[1];
      output.points[idx].normal_z = n[2];

      flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                  output.points[idx].normal[0],
                                  output.points[idx].normal[1],
                                  output.points[idx].normal[2]);
    }
  }

  }
  else
  {
#ifdef _OPENMP
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
#endif
     // Iterating over the entire index vector
    for (int idx = 0; idx < static_cast<int> (indices_->size ()); ++idx)
    {
      if (!isFinite ((*input_)[(*indices_)[idx]]) ||
          this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
      {
        output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();

        output.is_dense = false;
        continue;
      }

      Eigen::Vector4f n;
      pcl::computePointNormal<PointInT> (*surface_, nn_indices,
                                         n,
                                         output.points[idx].curvature);
                          
      output.points[idx].normal_x = n[0];
      output.points[idx].normal_y = n[1];
      output.points[idx].normal_z = n[2];

      flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                  output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2]);
    }
 }
}

#define PCL_INSTANTIATE_NormalEstimationOMP(T,NT) template class PCL_EXPORTS pcl::NormalEstimationOMP<T,NT>;

#endif    // PCL_FEATURES_IMPL_NORMAL_3D_OMP_H_

