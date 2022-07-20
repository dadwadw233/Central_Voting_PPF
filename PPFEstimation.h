//
// Created by yyh on 22-7-20.
//

#ifndef CENTRAL_VOTING_PPFESTIMATION_H
#define CENTRAL_VOTING_PPFESTIMATION_H
#include "Eigen/Core"
#include "HashMap.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
class PPFEstimation {
 public:
  void compute(const pcl::PointCloud<pcl::PointNormal>::Ptr &input_point_normal,
               pcl::PointCloud<pcl::PPFSignature>::Ptr &output_cloud,
               Hash::Ptr &hash_map);


  PPFEstimation &operator=(const PPFEstimation &) = delete;
  PPFEstimation(const PPFEstimation &) = delete;

 private:
};

#endif  // CENTRAL_VOTING_PPFESTIMATION_H
