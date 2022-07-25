//
// Created by yyh on 22-7-20.
//
#include "PPFEstimation.h"
#include "omp.h"
#include "chrono"
void PPFEstimation::compute(
    const pcl::PointCloud<pcl::PointNormal>::Ptr &input_point_normal,
    pcl::PointCloud<pcl::PPFSignature>::Ptr &output_cloud, Hash::Ptr &hash_map) {

  /*std::shared_ptr<pcl::PPFSignature> feature = std::make_shared<pcl::PPFSignature>();
  std::shared_ptr<Hash::HashData>data = std::make_shared<Hash::HashData>();
  std::shared_ptr<Hash::HashKey>key = std::make_shared<Hash::HashKey>();*/
  pcl::PPFSignature feature;
  Hash::HashData data;
  Hash::HashKey key;
  auto tp1 = std::chrono::steady_clock::now();

  for (auto i = 0;i<input_point_normal->size();i++) {
#pragma omp parallel shared(input_point_normal, output_cloud, hash_map,cout,i) private(feature, data, key) default(none)
      {
#pragma omp for
    for (auto j = 0; j < input_point_normal->size(); j++) {
      if (i == j) {
        continue;
      } else {
        Eigen::Vector4f p1(input_point_normal->points[i].x,
                           input_point_normal->points[i].y,
                           input_point_normal->points[i].z, 0.0f);

        Eigen::Vector4f p2(input_point_normal->points[j].x,
                           input_point_normal->points[j].y,
                           input_point_normal->points[j].z, 0.0f);
        Eigen::Vector4f n1(input_point_normal->points[i].normal);
        Eigen::Vector4f n2(input_point_normal->points[j].normal);
        Eigen::Vector4f delta = p2 - p1;

      delta = p2-p1;
        float f4 = delta.norm();

        // normalize
        delta /= f4;

        float f1 = n1[0] * delta[0] + n1[1] * delta[1] + n1[2] * delta[2];

        float f2 = n1[0] * delta[0] + n2[1] * delta[1] + n2[2] * delta[2];

        float f3 = n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2];

        feature.f1 = f1;
        feature.f2 = f2;
        feature.f3 = f3;
        feature.f4 = f4;


        data.Or = (std::make_pair(
            n1.cross3(delta), std::make_pair(n1.cross3(n1.cross3(delta)), n1)));
        data.Ot = (std::make_pair(
            n2.cross3(delta), std::make_pair(n2.cross3(n2.cross3(delta)), n2)));


        key.k1 = static_cast<int>(std::floor(f1 / angle_discretization_step));
        key.k2 = static_cast<int>(std::floor(f2 / angle_discretization_step));
        key.k3 = static_cast<int>(std::floor(f3 / angle_discretization_step));
        key.k4 =
            static_cast<int>(std::floor(f4 / distance_discretization_step));
        //std::cout << i << " " << j << std::endl;
        #pragma omp critical
        hash_map->addInfo(key, data);

        #pragma omp critical
        output_cloud->points.push_back(feature);

        //key.reset();
        //data.reset();
        //feature.reset();
      }
    }
  }
}
#pragma omp barrier
         auto tp2 = std::chrono::steady_clock::now();
         std::cout <<"need "<< std::chrono::duration_cast<std::chrono::milliseconds>(tp2 - tp1).count() << "ms to process PPF" << std::endl;
}


void PPFEstimation::setDiscretizationSteps(const float &angle_discretization_step, const float &distance_discretization_step) {
  this->angle_discretization_step = angle_discretization_step;
  this->distance_discretization_step = distance_discretization_step;
}
PPFEstimation::PPFEstimation() {}
