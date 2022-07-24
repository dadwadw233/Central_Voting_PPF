//
// Created by yyh on 22-7-11.
//
#ifndef CENTRAL_VOTING_CENTRALVOTING_H
#define CENTRAL_VOTING_CENTRALVOTING_H
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

#include <pcl/features/fpfh.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Core>
#include <fstream>
#include <limits>
#include <pcl/search/impl/search.hpp>
#include <utility>
#include <vector>
#include "Eigen/Core"
#include "HashMap.h"
#include "pcl/console/print.h"
#include "pcl/point_cloud.h"
#include "pcl/point_representation.h"
#include "pcl/registration/registration.h"
#include "pcl/visualization/cloud_viewer.h"

class CentralVoting {
 public:
  CentralVoting(pcl::PointCloud<pcl::PointXYZ>::Ptr input_scene,
                pcl::PointCloud<pcl::PointXYZ>::Ptr input_model)
      : scene(std::move(input_scene)) {
    model_set.push_back(std::move(input_model));
  }
  CentralVoting(pcl::PointCloud<pcl::PointXYZ>::Ptr input_scene,
                std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> input_models)
      : scene(std::move(input_scene)), model_set(std::move(input_models)) {}

  explicit CentralVoting(pcl::PointCloud<pcl::PointXYZ>::Ptr input_model) {
    this->model_set.push_back(std::move(input_model));
  }

  bool AddModel(pcl::PointCloud<pcl::PointXYZ>::Ptr input_model);

  pcl::PointCloud<pcl::PointNormal>::Ptr getPointNormal(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

  bool CenterExtractorAll();

  void InitTripleSet();

  static void GenerateBound(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
      pcl::PointXYZ &max_point, pcl::PointXYZ &min_point);

  pcl::PointCloud<pcl::PointNormal>::Ptr DownSample(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud) const;

  pcl::PointCloud<pcl::PointXYZ>::Ptr SimpleDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud);

  void Solve();

  void EstablishHashMap();

  void EstablishLRF();

  void test();  //测试入口函数

  void setNormalEstimationRadius(const float &radius);

  void setAngleThreshold(const float &angle);

  void setDownSampleStep(const float &step);

  void setSimpleDownSampleLeaf(const Eigen::Vector4f &subsampling_leaf_size);

  CentralVoting &operator=(const CentralVoting &) = delete;
  CentralVoting(const CentralVoting &) = delete;

 private:
  void CenterExtractor(int index = 0);
  int maxModelNum = 10;
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> model_set;
  std::vector<std::vector<pcl::PointXYZ>> triple_set;
  pcl::PointCloud<pcl::PointNormal>::Ptr scene_subsampled;
  float step;
  Eigen::Vector4f subsampling_leaf_size;
  float AngleThreshold;
  float distanceThreshold;
  float normalEstimationRadius;
};
#endif  // CENTRAL_VOTING_CENTRALVOTING_H
