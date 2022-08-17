//
// Created by yyh on 22-7-20.
//
#include "PPFEstimation.h"
#include "chrono"
#include "omp.h"
void PPFEstimation::compute(
    const pcl::PointCloud<pcl::PointNormal>::Ptr &input_point_normal,
    pcl::PointCloud<pcl::PPFSignature>::Ptr &output_cloud,
    Hash::Ptr &hash_map) {
  /*std::shared_ptr<pcl::PPFSignature> feature =
  std::make_shared<pcl::PPFSignature>(); std::shared_ptr<Hash::HashData>data =
  std::make_shared<Hash::HashData>(); std::shared_ptr<Hash::HashKey>key =
  std::make_shared<Hash::HashKey>();*/
  pcl::PPFSignature feature{};
  std::pair<Hash::HashKey, Hash::HashData> data{};
  // Hash::HashData data;
  // Hash::HashKey key;
  Eigen::Vector3f p1{};
  Eigen::Vector3f p2{};
  Eigen::Vector3f n1{};
  Eigen::Vector3f n2{};
  Eigen::Vector3f delta{};

  auto tp1 = std::chrono::steady_clock::now();

  for (auto i = 0; i < input_point_normal->size(); ++i) {
#pragma omp parallel shared(input_point_normal, output_cloud, hash_map, cout, \
                            i) private(feature, data, p1, p2, n1, n2,         \
                                       delta) default(none)
    {
#pragma omp for
      for (auto j = 0; j < input_point_normal->size(); ++j) {
        if (i == j) {
          continue;
        } else {
          /*Eigen::Vector4f p1(input_point_normal->points[i].x,
                             input_point_normal->points[i].y,
                             input_point_normal->points[i].z, 0.0f);

          Eigen::Vector4f p2(input_point_normal->points[j].x,
                             input_point_normal->points[j].y,
                             input_point_normal->points[j].z, 0.0f);
          Eigen::Vector4f n1(input_point_normal->points[i].normal);
          Eigen::Vector4f n2(input_point_normal->points[j].normal);
          Eigen::Vector4f delta = p2 - p1;     */
          p1 << input_point_normal->points[i].x,
              input_point_normal->points[i].y, input_point_normal->points[i].z;
          p2 << input_point_normal->points[j].x,
              input_point_normal->points[j].y, input_point_normal->points[j].z;
          n1 << input_point_normal->points[i].normal_x,
              input_point_normal->points[i].normal_y,
              input_point_normal->points[i].normal_z;
          n2 << input_point_normal->points[j].normal_x,
              input_point_normal->points[j].normal_y,
              input_point_normal->points[j].normal_z;
          // std::cout<<input_point_normal->points[j]<<std::endl;
          delta = p2 - p1;//pt-pr
          float f4 = delta.norm();
/*
          if(f4<250)
          {
            continue;
          }*/
          //std::cout<<f4<<std::endl;
          // normalize
          delta /= f4;

          float f1 = n1[0] * delta[0] + n1[1] * delta[1] + n1[2] * delta[2];

          float f2 = n1[0] * delta[0] + n2[1] * delta[1] + n2[2] * delta[2];

          float f3 = n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2];

          /*float f1 = n1.x() * delta.x() + n1.y()  * delta.y() + n2.z()  *
          delta.z();

          float f2 = n1.x() * delta.x() + n2.y()  * delta.y() + n2.z()  *
          delta.z();

          float f3 = n1.x() * n2.x() + n1.y()  * n2.y()  + n1.z()  * n2.z() ;
           */
          feature.f1 = f1;
          feature.f2 = f2;
          feature.f3 = f3;
          feature.f4 = f4;
          feature.alpha_m = 0.0f;
          data.second.Or =
              (std::make_pair(n1.cross(delta)/(n1.cross(delta)).norm(),
                              std::make_pair(n1.cross(n1.cross(delta))/(n1.cross(n1.cross(delta))).norm(), n1/n1.norm())));

          data.second.Ot =
              (std::make_pair(n2.cross(delta)/(n2.cross(delta)).norm(),
                              std::make_pair(n2.cross(n2.cross(delta))/(n2.cross(n2.cross(delta))).norm(), n2/n2.norm())));

          data.first.k1 =
              static_cast<int>(std::floor(f1 / angle_discretization_step));
          data.first.k2 =
              static_cast<int>(std::floor(f2 / angle_discretization_step));
          data.first.k3 =
              static_cast<int>(std::floor(f3 / angle_discretization_step));
          data.second.dist =
              static_cast<int>(std::floor(f4 / distance_discretization_step));

          data.second.r = input_point_normal->points[i];
          data.second.t = input_point_normal->points[j];
// std::cout << i << " " << j << std::endl;
#pragma omp critical
          hash_map->addInfo(data);

#pragma omp critical
          output_cloud->points.push_back(feature);
        }
      }
    }
  }
#pragma omp barrier
  auto tp2 = std::chrono::steady_clock::now();
  std::cout << "need "
            << std::chrono::duration_cast<std::chrono::milliseconds>(tp2 - tp1)
                   .count()
            << "ms to process PPF" << std::endl;
}

void PPFEstimation::setDiscretizationSteps(
    const float &angle_discretization_step,
    const float &distance_discretization_step) {
  this->angle_discretization_step = angle_discretization_step;
  this->distance_discretization_step = distance_discretization_step;
}
PPFEstimation::PPFEstimation() {}
