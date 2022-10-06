//
// Created by yyh on 22-7-12.
//

#ifndef CENTRAL_VOTING_SMARTDOWNSAMPLE_H
#define CENTRAL_VOTING_SMARTDOWNSAMPLE_H
#define PCL_NO_PRECOMPILE
#include <pcl/features/normal_3d.h>
#include <pcl/features/ppf.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/ppf_registration.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "Eigen/Core"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>  //pcl控制台解析
#include <pcl/console/parse.h>
#include <pcl/features/fpfh_omp.h>  //包含fpfh加速计算的omp(多核并行计算)
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h>  //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h>  //随机采样一致性去除
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tbb/concurrent_vector.h>
#include <Eigen/Core>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <thread>
#include "pcl/features/moment_of_inertia_estimation.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/filters/filter.h"
#include "pcl/search/kdtree.h"
#include "pcl/impl/point_types.hpp"
#include <pcl/common/common.h>

#include "queue"


class SmartDownSample {
 public:
  SmartDownSample(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                  const std::pair<double, double> x_range,
                  const std::pair<double, double> y_range,
                  const std::pair<double, double> z_range, const float &step,
                  const float &angleThreshold, const float &distanceThreshold)
      : input_cloud(input_cloud),
        x_range(x_range),
        y_range(y_range),
        z_range(z_range),
        step(step),
        angleThreshold(angleThreshold),
        distanceThreshold(distanceThreshold){};

  /** \brief Smart down sample method, if the angle between two point normals
   * bigger than angle threshold, this two point will also be sampled
   * \param[out] the subsampled point cloud
   */
  pcl::PointCloud<pcl::PointNormal>::Ptr compute();

  /** \brief Set the radius of normal estimation
   * \param[in] radius
   */
  void setRadius(float data);

  void setKSearch(const int data);
  /** \brief Calculate the distance between two points
   * \param[in] points
   */
  template <class T>
  float calculateDistance(T &pointA, T &pointB);

  template <class T>
  float calculateDistance(T &pointA, pcl::PointNormal &pointB);

  void setIsdense(const bool &data);

  void setViewPoint(const Eigen::Vector3f &view_point_){
    this->view_point = view_point_;
    isSetViewPoint = true;
  }

  void setViewPoint(const Eigen::Vector3f &&view_point_){
    this->view_point = view_point_;
    isSetViewPoint  = true;
  }

  void setReverse(const bool &flag){
    this->reverse = flag;
  }


 private:
  decltype(auto) getMeanPointNormal(const std::vector<pcl::PointNormal> &cluster){
    pcl::PointNormal Mean{};
    for(auto &i:cluster){
      Mean.x+=i.x;
      Mean.y+=i.y;
      Mean.z+=i.z;
      Mean.normal_x+=i.normal_x;
      Mean.normal_y+=i.normal_y;
      Mean.normal_z+=i.normal_z;
      Mean.curvature+=i.curvature;//对曲率做了平均处理
    }
    auto num = cluster.size();
    Mean.x/=num;
    Mean.y/=num;
    Mean.z/=num;
    Mean.curvature/=num;
    Eigen::Vector3f normal_{Mean.normal_x,Mean.normal_y,Mean.normal_z};
    normal_ = normal_.normalized();
    Mean.normal_x = normal_[0];
    Mean.normal_y = normal_[1];
    Mean.normal_z = normal_[2];
    return Mean;
  }
  struct data{
    int i;
    int j;
    pcl::PointNormal Mean{};
    data(const int &i_, const int &j_, const pcl::PointNormal &Mean_ ):i(i_), j(j_), Mean(Mean_){}
  };
  struct cmp {
    bool operator()(data a, data b) {
      if (a.i == b.i)
        return a.j > b.j;
      else
        return a.i > b.i;
    }
  };
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud;
  std::pair<double, double> x_range;
  std::pair<double, double> y_range;
  std::pair<double, double> z_range;
  std::priority_queue<data, std::vector<data>, cmp>q;
  float step;
  float angleThreshold, distanceThreshold;
  float normal_estimation_search_radius;
  Eigen::Vector3f view_point{};
  int normal_estimation_search_k_points;
  bool isSetRadius = false;
  bool isSetPoints = false;
  bool isdense = false;
  bool reverse = false;
  bool isSetViewPoint = false;
};

#endif  // CENTRAL_VOTING_SMARTDOWNSAMPLE_H
