//
// Created by yyh on 22-7-12.
//

#ifndef CENTRAL_VOTING_SMARTDOWNSAMPLE_H
#define CENTRAL_VOTING_SMARTDOWNSAMPLE_H
#include "Eigen/Core"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
class SmartDownSample {
 public:
  SmartDownSample(const pcl::PointCloud<pcl::PointXYZ>::Ptr  & input_cloud,
                  const std::pair<double, double> x_range,
                  const std::pair<double, double> y_range,
                  const std::pair<double, double> z_range, const float &step,
                  const float angleThreshold, const float &distanceThreshold)
      : input_cloud(input_cloud),
        x_range(x_range),
        y_range(y_range),
        z_range(z_range),
        step(step),
        angleThreshold(angleThreshold),
        distanceThreshold(distanceThreshold){};

  pcl::PointCloud<pcl::PointXYZ>::Ptr compute();

 private:
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud;
  std::pair<double, double> x_range;
  std::pair<double, double> y_range;
  std::pair<double, double> z_range;
  float step;
  float angleThreshold, distanceThreshold;
};

#endif  // CENTRAL_VOTING_SMARTDOWNSAMPLE_H
