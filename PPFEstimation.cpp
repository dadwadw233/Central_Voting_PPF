//
// Created by yyh on 22-7-20.
//
#include "PPFEstimation.h"
#include "omp.h"
void PPFEstimation::compute(
    const pcl::PointCloud<pcl::PointNormal>::Ptr &input_point_normal,
    pcl::PointCloud<pcl::PPFSignature>::Ptr &output_cloud, Hash::Ptr &hash_map) {
  for (auto i : *input_point_normal) {
    for (auto j : *input_point_normal) {
      if (i.x == j.x && i.y == j.y && i.z == j.z) {
        continue;
      } else {
        Eigen::Vector4f p1(i.x, i.y, i.z, 0.0f);
        Eigen::Vector4f p2(j.x, j.y, j.z, 0.0f);
        Eigen::Vector4f n1(i.normal);
        Eigen::Vector4f n2(j.normal);
        Eigen::Vector4f delta = p2 - p1;

        float f4 = delta.norm();

        // normalize
        delta /= f4;

        float f1 = n1[0] * delta[0] + n1[1] * delta[1] + n1[2] * delta[2];

        float f2 = n1[0] * delta[0] + n2[1] * delta[1] + n2[2] * delta[2];

        float f3 = n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2];

        pcl::PPFSignature feature{};

        feature.f1 = f1;
        feature.f2 = f2;
        feature.f3 = f3;
        feature.f4 = f4;

        Hash::HashData data;
        data.Or = (std::make_pair(
            n1.cross3(delta), std::make_pair(n1.cross3(n1.cross3(delta)), n1)));
        data.Ot = (std::make_pair(
            n2.cross3(delta), std::make_pair(n2.cross3(n2.cross3(delta)), n2)));
        Hash::HashKey key{};
        key.k1 = static_cast<int>(std::floor(f1/angle_discretization_step));
        key.k2 = static_cast<int>(std::floor(f2/angle_discretization_step));
        key.k3 = static_cast<int>(std::floor(f3/angle_discretization_step));
        key.k4 = static_cast<int>(std::floor(f4/distance_discretization_step));
        hash_map->addInfo(key,data);
        output_cloud->push_back(feature);
      }
    }
  }
}
void PPFEstimation::setDiscretizationSteps(const float &angle_discretization_step, const float &distance_discretization_step) {
  this->angle_discretization_step = angle_discretization_step;
  this->distance_discretization_step = distance_discretization_step;
}
PPFEstimation::PPFEstimation() {}
