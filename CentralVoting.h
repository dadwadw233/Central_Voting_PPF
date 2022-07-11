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

#include <utility>
#include "HashMap.h"
#include "pcl/console/print.h"
#include "pcl/point_cloud.h"
#include "pcl/point_representation.h"
#include "pcl/registration/registration.h"
#include "pcl/visualization/cloud_viewer.h"

#include "Eigen/Core"

class CentralVoting {
 public:
  CentralVoting(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_scene,
                pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_model)
      : scene(std::move(input_scene)) {
    model_set.push_back(std::move(input_model));
  }
  CentralVoting(
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_scene,
      std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> input_models)
      : scene(std::move(input_scene)), model_set(std::move(input_models)) {}

  explicit CentralVoting(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_model) {
    this->model_set.push_back(std::move(input_model));
  }

  bool AddModel(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_model);

  bool CenterExtractor(int index = 0);

  void GenerateBound();

  void DownSample();

  void EstablishPPF();

  void EstablishHashMap();

  void EstablishLRF();

  CentralVoting &operator=(const CentralVoting &) = delete;
  CentralVoting(const CentralVoting &) = delete;

 private:
  int maxModelNum = 10;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr scene;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> model_set;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr scene_subsampled;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr model_subsampled;
};
#endif  // CENTRAL_VOTING_CENTRALVOTING_H
