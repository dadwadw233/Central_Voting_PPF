//
// Created by yyh on 22-7-26.
//
#include "pcl/registration/registration.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "HashMap.h"
#include <pcl/segmentation/extract_clusters.h>
#ifndef CENTRAL_VOTING_PPFREGISTRATION_H
#define CENTRAL_VOTING_PPFREGISTRATION_H

class PPFRegistration{
 public:
  PPFRegistration();

  void compute();

  Eigen::Affine3f getFinalTransformation();

  template <class T>
  typename pcl::PointCloud<T>::Ptr aligen(const typename pcl::PointCloud<T>::Ptr &input);

  void setSceneReferencePointSamplingRate(const float &scene_reference_point_sampling_rate);

  void setPositionClusteringThreshold(const float &clustering_position_diff_threshold);

  void setRotationClusteringThreshold(const float &clustering_rotation_diff_threshold);

  void setSearchMap(const Hash::Ptr &searchMap);

  void setInputSource(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);

  void setInputTarget(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);

  void setDiscretizationSteps(const float &angle_discretization_step,
                              const float &distance_discretization_step);

  void setDobj(const float &data);

  void setModelTripleSet(const std::vector<pcl::PointXYZ>&triple_set);

  void vote(const int &key,const Eigen::Affine3f &T);

  void establishVoxelGrid();

  PPFRegistration &operator=(const PPFRegistration &) = delete;
  PPFRegistration(const PPFRegistration &) = delete;

 private:
  struct data{
    Eigen::Affine3f T;
    int value;
    data(const Eigen::Affine3f &T_, const int value_){
      T = T_;
      value = value_;
    }
  };
  float scene_reference_point_sampling_rate{};
  float clustering_position_diff_threshold{};
  float clustering_rotation_diff_threshold{};
  std::pair<double, double> x_range;
  std::pair<double, double> y_range;
  std::pair<double, double> z_range;
  pcl::PointCloud<pcl::PointNormal>::Ptr model_cloud_with_normal;
  pcl::PointCloud<pcl::PointNormal>::Ptr scene_cloud_with_normal;
  Eigen::Affine3f finalTransformation;
  Hash::Ptr searchMap;
  std::vector<pcl::PointXYZ>triple_set;
  float angle_discretization_step;
  float distance_discretization_step;
  float d_obj;
  std::unordered_map<int, struct data>map;
};

#endif  // CENTRAL_VOTING_PPFREGISTRATION_H
