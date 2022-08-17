//
// Created by yyh on 22-7-26.
//
#include <pcl/segmentation/extract_clusters.h>
#include "HashMap.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/registration/registration.h"
#ifndef CENTRAL_VOTING_PPFREGISTRATION_H
#define CENTRAL_VOTING_PPFREGISTRATION_H

class PPFRegistration {
 public:
  PPFRegistration();

  void compute();

  Eigen::Affine3f getFinalTransformation();

  template <class T>
  typename pcl::PointCloud<T>::Ptr aligen(
      const typename pcl::PointCloud<T>::Ptr &input);

  void setSceneReferencePointSamplingRate(
      const float &scene_reference_point_sampling_rate);

  void setPositionClusteringThreshold(
      const float &clustering_position_diff_threshold);

  void setRotationClusteringThreshold(
      const float &clustering_rotation_diff_threshold);

  void setSearchMap(const Hash::Ptr &searchMap);

  void setInputSource(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);

  void setInputTarget(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);

  void setDiscretizationSteps(const float &angle_discretization_step,
                              const float &distance_discretization_step);

  void setDobj(const float &data);

  void setModelTripleSet(const std::vector<pcl::PointXYZ> &triple_set);


  void establishVoxelGrid();

  PPFRegistration &operator=(const PPFRegistration &) = delete;
  PPFRegistration(const PPFRegistration &) = delete;

 private:
  struct scale{

  };
  struct data_ {
    std::vector<Eigen::Affine3f> T_set;
    int value = 0;
    data_(const Eigen::Affine3f &T_, const int value_) {
      T_set.push_back(T_);
      value += value_;
    }
  };
  struct key_ {
    int index_c;
    int index_aux_1;
    int index_aux_2;
    key_(int center, int first_aux, int second_aux){
      index_c = center;
      index_aux_1 = first_aux;
      index_aux_2 = second_aux;
    }
    bool operator==(const key_ &k) const {
      return index_c == k.index_c && index_aux_1 == k.index_aux_1 &&
             index_aux_2 == k.index_aux_2;
    }
  };
  struct data{
    Eigen::Affine3f T;
    int value;
    data(const Eigen::Affine3f &T_, const int &value_):T(T_), value(value_){}
  };
  struct cmp{
    bool operator()(data a, data b){
      if(a.value == b.value) return a.value<=b.value;
      else return a.value<b.value;
    }
  };
  void vote(const key_ &key, const Eigen::Affine3f &T);

  void vote(const int &key, const Eigen::Affine3f &T);

  decltype(auto) HypoVerification(const Eigen::Affine3f &T);

  decltype(auto) HypoVerification(const Eigen::Matrix4f &T);

  template <class T>
  float calculateDistance(T &pointA, T &pointB);

  decltype(auto) getMeanMatrix(const std::vector<Eigen::Affine3f> &T_set){
    Eigen::Matrix4f temp;
    temp<<0,0,0,0,
        0,0,0,0,
        0,0,0,0,
        0,0,0,0;
    for(auto i:T_set){
      temp+=i.matrix();
    }
    temp/=T_set.size();
    return temp;
  }
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
  std::vector<pcl::PointXYZ> triple_set;
  float angle_discretization_step;
  float distance_discretization_step;
  float d_obj;
  struct hash_cal {
    size_t operator()(const key_ &k) const {
      return std::hash<int>()(k.index_c) ^
             (std::hash<int>()(k.index_aux_1) << 1) ^
             (std::hash<int>()(k.index_aux_2) << 2) ^
             (std::hash<int>()(k.index_c) << 3) ^
             (std::hash<int>()(k.index_aux_1) << 4) ^
             (std::hash<int>()(k.index_aux_2) << 5);
      // return std::hash<int>()(k.k1);
    }
  };
  std::unordered_map<key_, data_, hash_cal> map_;
  std::unordered_map<int, data_>map_center;
  std::priority_queue<data, std::vector<data>, cmp>T_queue;
  //std::priority_queue<data, std::vector<data>, cmp>T_queue;
};

#endif  // CENTRAL_VOTING_PPFREGISTRATION_H
