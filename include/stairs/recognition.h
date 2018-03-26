#ifndef RECOGNITION
#define RECOGNITION

#include <stairs/regions.h>
#include <stairs/StairVector.h>

#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <unsupported/Eigen/NonLinearOptimization>

#include <pcl/visualization/pcl_visualizer.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::Normal Normal;
typedef pcl::PointXYZRGB PointTC;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointTC> PointCloudC;

typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VectorX;

class recognition {
public:

	recognition();

    regions segments;

    bool testBool;
    bool wtf;

    bool parFlag;
    bool ndFlag;
    bool pdFlag;
    float parAngle;
    bool widthFlag;

    bool predifinedValues;
    double preDefDepth;
    double preDefHeight;
    double preDefWidth;

    bool floorInformation;

    Eigen::Vector2f nDistance;
    Eigen::Vector2f pDistance;

    inline void setFloorFlag (bool input)
    {
    	floorInformation = input;
    }

    inline void setParFlag(bool input)
    {
        parFlag = input;
    }

    inline void setWidthFlag(bool input)
    {
    	widthFlag = input;
    }

    inline void setPriorKnowledge(bool input)
    {
    	predifinedValues = input;
    }

    inline void setStairDepth(double input)
    {
    	preDefDepth = input/100;
    }

    inline void setStairHeight(double input)
    {
    	preDefHeight = input/100;
    }

    inline void setStairWidth(double input)
    {
    	preDefWidth = input/100;
    }

    inline void setParAngle(float input)
    {
        parAngle = cos(input/180*M_PI);
    }

    inline void setNDFlag(bool input)
    {
        ndFlag = input;
    }

    inline void setPDFlag(bool input)
    {
        pdFlag = input;
    }

    inline void setNDistance(Eigen::Vector2f input)
    {
        nDistance = input;
    }

    inline void setPDistance(Eigen::Vector2f input)
    {
        pDistance = input;
    }

    inline void setInputRegions(regions input_)
    {
        segments = input_;
        inputRegions = input_;
    }

    void filter();
    void filterCircular();

    regions inputRegions;
    regions indicatorScRegions;
    regions indicatorSwRegions;

    regions stairTreadRegions;
    regions stairRiseRegions;

    regions tempOutput;
    regions tempComponentOut;

    regions stairTreads;
    regions stairRises;

    regions stairParts;

    float maxStairRiseDist;
    float maxStairRiseHDist;
    float maxStairRiseAngle;
    float maxStairTreadDist;

    double memTime;
    double whole_time;
    double widthTime;
    double sortTime;
    double getWTime;

    bool stairRailFlag;

    int basePart;

    Eigen::Vector2f widthReqVec;

    Eigen::Vector3f distVec;
    Eigen::Vector3f startSearchPoint;
    Eigen::Vector3f stairPos;
    Eigen::Vector2i stepAmount;

    StairVector stairs;
    int stairCount;

    std::vector<int> addedLabel;
    std::vector<int> globalAddedLabel;

    bool updateFlag;
    bool optimizeFlag;

    float angleDiff;
    float distCircCent;
    bool clockWise;
    bool graphMeth;

    inline void setGraphMeth(bool input)
    {
    	graphMeth = input;
    }

    inline void setUpdateFlag(bool input)
    {
    	updateFlag = input;
    }

    inline void setOptimizeFlag(bool input)
    {
    	optimizeFlag = input;
    }

    inline void setWidthReq(Eigen::Vector2f input)
    {
    	widthReqVec = input;
    }

    inline void setStairRailFlag(bool input)
    {
        stairRailFlag = input;
    }

    inline void setStairTreadRegions(regions input)
    {
        stairTreadRegions = input;
    }

    inline void setStairRiseRegions(regions input)
    {
        stairRiseRegions = input;
    }

    void run(StairVector& output);

    void finalize(Stairs& input);


    void check();

    bool isStairRiseMatch(int regPos, int stairNo);
    bool isStairTreadMatch(int regPos, int stairNo);
    void findStairRail(Stairs& input);
    void findStairRailCirc(Stairs& input);
    void expandSearch();
    bool widthReq(segmentPatch& testPatch);
    bool widthReqCirc(segmentPatch& testPatch, int stairNo);

    Eigen::Vector3f getStairScore();
    Eigen::Matrix<float,5,1> getStairScoreNew();
    Eigen::Vector3f getStairScoreCirc();
    void optimizeCoefficients();

//    VectorX computeDistance(Eigen::Matrix<float,5,1> estimateVector, regions nstairRises, regions nstairTreads) const;
    VectorX computeDistanceNew(Eigen::Matrix<float,5,1> estimateVector, regions nstairRises, regions nstairTreads) const;

    void find();

    template<typename _Scalar, int NX=Eigen::Dynamic, int NY=Eigen::Dynamic>
    struct Functor
    {
      typedef _Scalar Scalar;
      enum
      {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
      };
      typedef Eigen::Matrix<_Scalar,InputsAtCompileTime,1> InputType;
      typedef Eigen::Matrix<_Scalar,ValuesAtCompileTime,1> ValueType;
      typedef Eigen::Matrix<_Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

      /** \brief Empty Construtor. */
      Functor () : m_data_points_ (ValuesAtCompileTime) {}

      /** \brief Constructor
        * \param[in] m_data_points number of data points to evaluate.
        */
      Functor (int m_data_points) : m_data_points_ (m_data_points) {}

      /** \brief Destructor. */
      virtual ~Functor () {}

      /** \brief Get the number of values. */
      int
      values () const { return (m_data_points_); }

      protected:
        int m_data_points_;
    };

    struct OptimizationFunctor : public Functor<float>
    {
      using Functor<float>::values;

      /** Functor constructor
        * \param[in] m_data_points the number of data points to evaluate
        * \param[in,out] estimator pointer to the estimator object
        */
      OptimizationFunctor (int m_data_points,
                           const recognition *estimator)
        :  Functor<float> (m_data_points), estimator_ (estimator)
      {}

      /** Copy constructor
        * \param[in] src the optimization functor to copy into this
        */
      inline OptimizationFunctor (const OptimizationFunctor &src) :
        Functor<float> (src.m_data_points_), estimator_ ()
      {
        *this = src;
      }

      /** Copy operator
        * \param[in] src the optimization functor to copy into this
        */
      inline OptimizationFunctor&
      operator = (const OptimizationFunctor &src)
      {
        Functor<float>::operator=(src);
        estimator_ = src.estimator_;
        return (*this);
      }

      /** \brief Destructor. */
      virtual ~OptimizationFunctor () {}

      /** Fill fvec from x. For the current state vector x fill the f values
        * \param[in] x state vector
        * \param[out] fvec f values vector
        */
      int
      operator () (const VectorX &x, VectorX &fvec) const;

      const recognition *estimator_;
    };

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // RECOGNITION

