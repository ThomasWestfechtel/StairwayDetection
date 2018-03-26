# StairwayDetection

![alt text](https://github.com/ThomasWestfechtel/StairwayDetection/blob/master/pics/resultExample.png "Staiway Detection Example")

Repository for Stairway Detection using PointCloud Data

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


The processing steps of the algorihm are explained in "3D graph based stairway detection and localization for mobile robots" (http://ieeexplore.ieee.org/document/7759096/).

![alt text](https://github.com/ThomasWestfechtel/StairwayDetection/blob/master/pics/stairGraph.png "Graph-based Detection")
