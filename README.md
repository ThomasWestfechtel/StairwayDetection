# Graph-based Stairway Detection using PointCloud Data

![alt text](https://github.com/ThomasWestfechtel/StairwayDetection/blob/master/pics/resultExample.png "Staiway Detection Example")

To install the repository:

Go to main directory

```
mkdir build
cd build

cmake ..
make -j8
```

2 example situation are given in the repository. To test them use:

```
./stair_det ../examples/13-Depth.pcd ../examples/13-Result.pcd
./stair_det ../examples/25-Depth.pcd ../examples/25-Result.pcd
```

Depending on the accuracy of the employed LIDAR some parameters need to be tuned accordingly. Most important are the parameters of the region growing algorithm in src/regiongrowing.h

This is the stand-alone version of the Stairway Detection. A version including a GUI can be found at:
https://github.com/ThomasWestfechtel/StairwayDetectionGUI

The processing steps of the algorihm are explained in our papers:
"Robust stairway-detection and localization method for mobile robots using a graph-based model and competing initializations" (https://journals.sagepub.com/doi/full/10.1177/0278364918798039) (IJRR)
and
"3D graph based stairway detection and localization for mobile robots" (http://ieeexplore.ieee.org/document/7759096/) (IROS).

![alt text](https://github.com/ThomasWestfechtel/StairwayDetection/blob/master/pics/stairGraph.png "Graph-based Detection")
