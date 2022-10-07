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

  void setSearchMap(const Hash::HashMap::Ptr &searchMap);

  void setInputSource(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);

  void setInputTarget(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);

  void setHypoSource(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);

  void setHypoTarget(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);

  void setDiscretizationSteps(const float &angle_discretization_step,
                              const float &distance_discretization_step);

  void setDobj(const float &data);

  void setModelTripleSet(const std::vector<pcl::PointXYZ> &triple_set);

  void establishVoxelGrid();

  void setGroundTruthTransform(const Eigen::Matrix4f &gt_){
    this->gt<<gt_;
  }

  PPFRegistration &operator=(const PPFRegistration &) = delete;
  PPFRegistration(const PPFRegistration &) = delete;

 private:
  double calculate_rotation_error(Eigen::Matrix3f& est, Eigen::Matrix3f& gt) {
    double tr = (est.transpose() * gt).trace();
    if (tr > 3) tr = 3;
    if (tr < -1) tr = -1;
    return acos((tr - 1) / 2) * 180.0 / M_PI;
  }

  double calculate_translation_error(Eigen::Vector3f& est, Eigen::Vector3f& gt) {
    Eigen::Vector3f t = est - gt;
    return sqrt(t.dot(t));
  }

  bool evaluation_est(Eigen::Matrix4f est/*估计*/, Eigen::Matrix4f gt/*真值*/, double re_thresh/*旋转误差阈值*/, double te_thresh/*平移误差阈值*/, double& RE/*旋转误差*/, double& TE/*平移误差*/) {
    Eigen::Matrix3f rotation_est, rotation_gt;
    Eigen::Vector3f translation_est, translation_gt;
    rotation_est = est.topLeftCorner(3, 3);
    rotation_gt = gt.topLeftCorner(3, 3);
    translation_est = est.block(0, 3, 3, 1);
    translation_gt = gt.block(0, 3, 3, 1);

    RE = calculate_rotation_error(rotation_est, rotation_gt);
    TE = calculate_translation_error(translation_est, translation_gt);
    if (0 <= RE && RE <= re_thresh && 0 <= TE && TE <= te_thresh)
    {
      return true;
    }
    return false;
  }

  struct data_ {
    Eigen::Quaternionf sumQ{0,0,0,0};
    Eigen::Vector3f sumt{0,0,0};
    int value = 0;
    data_(const Eigen::Affine3f &T_, const int value_) {
      auto q = Eigen::Quaternionf (T_.rotation());
      sumQ.x()+= q.x();
      sumQ.y()+= q.y();
      sumQ.z()+= q.z();
      sumQ.w()+= q.w();
      sumt.x()+= T_.translation().x();
      sumt.y()+= T_.translation().y();
      sumt.z()+= T_.translation().z();
      value += value_;
    }
  };
  struct key_ {
    int index_c;
    int index_aux_1;
    int index_aux_2;
    key_(int center, int first_aux, int second_aux) {
      index_c = center;
      index_aux_1 = first_aux;
      index_aux_2 = second_aux;
    }
    bool operator==(const key_ &k) const {
      return index_c == k.index_c && index_aux_1 == k.index_aux_1 &&
             index_aux_2 == k.index_aux_2;
    }
  };
  struct data {
    Eigen::Affine3f T;
    float value;
    data(const Eigen::Affine3f &T_, const float &value_) : T(T_), value(value_) {}
  };
  struct cmp {
    bool operator()(data a, data b) {
      if (a.value == b.value)
        return a.value <= b.value;
      else
        return a.value < b.value;
    }
  };
  void vote(const key_ &key, const Eigen::Affine3f &T);

  void vote(const int &key, const Eigen::Affine3f &T);

  decltype(auto) HypoVerification(const Eigen::Affine3f &T);

  decltype(auto) HypoVerification(const Eigen::Matrix4f &T);

  decltype(auto) ICVRHypoVerification(const Eigen::Matrix4f &T);

  template <class T>
  float calculateDistance(T &pointA, T &pointB);

  template <class T>
  float calculateDistanceP(T &pointA, T &pointB);


  decltype(auto) getMeanMatrix(const data_ &data) {
    Eigen::Quaternionf averQ {};
    averQ.x() = data.sumQ.x()/ data.value;
    averQ.y() = data.sumQ.y()/ data.value;
    averQ.z() = data.sumQ.z()/ data.value;
    averQ.w() = data.sumQ.w()/ data.value;
    Eigen::Vector3f t = data.sumt/data.value;
    Eigen::Affine3f R{averQ};
    Eigen::Matrix4f result{};
    result<<R.rotation()(0,0),R.rotation()(0,1),R.rotation()(0,2),t[0],
        R.rotation()(1,0),R.rotation()(1,1),R.rotation()(1,2),t[1],
        R.rotation()(2,0),R.rotation()(2,1),R.rotation()(2,2),t[2],
        0,0,0,1;
    return result;
  }
  float scene_reference_point_sampling_rate{};
  float clustering_position_diff_threshold{};
  float clustering_rotation_diff_threshold{};
  std::pair<double, double> x_range;
  std::pair<double, double> y_range;
  std::pair<double, double> z_range;
  pcl::PointCloud<pcl::PointNormal>::Ptr model_cloud_with_normal;
  pcl::PointCloud<pcl::PointNormal>::Ptr scene_cloud_with_normal;
  pcl::PointCloud<pcl::PointNormal>::Ptr hypo_scene_cloud;
  pcl::PointCloud<pcl::PointNormal>::Ptr hypo_model_cloud;
  Eigen::Affine3f finalTransformation;
  Eigen::Matrix4f gt{};
  Hash::HashMap::Ptr searchMap;
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
  std::unordered_map<int, data_> map_center;
  std::priority_queue<data, std::vector<data>, cmp> T_queue;
};

#endif  // CENTRAL_VOTING_PPFREGISTRATION_H
