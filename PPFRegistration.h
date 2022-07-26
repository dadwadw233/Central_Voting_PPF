//
// Created by yyh on 22-7-26.
//
#include "pcl/registration/registration.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "HashMap.h"
#ifndef CENTRAL_VOTING_PPFREGISTRATION_H
#define CENTRAL_VOTING_PPFREGISTRATION_H

class PPFRegistration{
 public:
  PPFRegistration();

  void compute();

  decltype(auto) getFinalTransformation();

  template <class T>
  typename pcl::PointCloud<T>::Ptr aligen(const typename pcl::PointCloud<T>::Ptr &input);

  void setSceneReferencePointSamplingRate(const float &scene_reference_point_sampling_rate);

  void setPositionClusteringThreshold(const float &clustering_position_diff_threshold);

  void setRotationClusteringThreshold(const float &clustering_rotation_diff_threshold);

  void setSearchMap(const Hash::Ptr &searchMap);

  void setInputSource(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);

  void setInputTarget(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);



  PPFRegistration &operator=(const PPFRegistration &) = delete;
  PPFRegistration(const PPFRegistration &) = delete;

 private:
  float scene_reference_point_sampling_rate{};
  float clustering_position_diff_threshold{};
  float clustering_rotation_diff_threshold{};
  pcl::PointCloud<pcl::PointNormal>::Ptr model_cloud_with_normal;
  pcl::PointCloud<pcl::PointNormal>::Ptr scene_cloud_with_normal;
  Eigen::Matrix4f finalTransformation;
  Hash::Ptr searchMap;
};

#endif  // CENTRAL_VOTING_PPFREGISTRATION_H
