//
// Created by yyh on 22-7-26.
//

#include "PPFRegistration.h"

PPFRegistration::PPFRegistration() {
  model_cloud_with_normal.reset( new pcl::PointCloud<pcl::PointNormal>());
  scene_cloud_with_normal.reset(new pcl::PointCloud<pcl::PointNormal>());
  searchMap.reset(new Hash::HashMap());
}
void PPFRegistration::setSceneReferencePointSamplingRate(const float &scene_reference_point_sampling_rate) {
  this->scene_reference_point_sampling_rate = scene_reference_point_sampling_rate;
}

void PPFRegistration::setPositionClusteringThreshold(const float &clustering_position_diff_threshold) {
  this->clustering_position_diff_threshold = clustering_position_diff_threshold;
}

void PPFRegistration::setRotationClusteringThreshold(const float &clustering_rotation_diff_threshold) {
  this->clustering_rotation_diff_threshold = clustering_rotation_diff_threshold;
}
void PPFRegistration::setInputTarget(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud) {
  this->model_cloud_with_normal = cloud;
}
void PPFRegistration::setInputSource(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud) {
  this->scene_cloud_with_normal = cloud;
}
void PPFRegistration::setSearchMap(const Hash::Ptr &searchMap) {
  this->searchMap = searchMap;
}
decltype(auto) PPFRegistration::getFinalTransformation() {
  return this->finalTransformation;
}
template<class T>
typename pcl::PointCloud<T>::Ptr PPFRegistration::aligen(const typename pcl::PointCloud<T>::Ptr &input) {
  typename pcl::PointCloud<T>::Ptr output = boost::make_shared<pcl::PointCloud<T>>();
    pcl::transformPointCloud(*input,*output,finalTransformation);
    return output;
}


void PPFRegistration::compute() {

}
