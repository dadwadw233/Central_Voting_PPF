//
// Created by yyh on 22-7-20.
//

#ifndef CENTRAL_VOTING_PPFESTIMATION_H
#define CENTRAL_VOTING_PPFESTIMATION_H
#include "Eigen/Core"
#include "HashMap.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "chrono"
#include "omp.h"

class PPFEstimation {
 public:
  PPFEstimation();
  decltype(auto) compute(
      const pcl::PointCloud<pcl::PointNormal>::Ptr &input_point_normal) {
    std::pair<Hash::HashKey, Hash::HashData> data{};
    int Nd = std::floor(this->dobj / this->distance_discretization_step) + 1;
    int Na = std::floor(float(M_PI) / this->angle_discretization_step) + 1;
    std::vector<
        std::vector<std::vector<std::vector<std::vector<Hash::HashData>>>>>
        map(Nd,
            std::vector<std::vector<std::vector<std::vector<Hash::HashData>>>>(
                Na,
                std::vector<std::vector<std::vector<Hash::HashData>>>(
                    Na, std::vector<std::vector<Hash::HashData>>(
                            Na, std::vector<Hash::HashData>(
                                    0, Hash::HashData{})))));  //产生静态数组

    // Hash::HashData data;
    // Hash::HashKey key;
    Eigen::Vector3f p1{};
    Eigen::Vector3f p2{};
    Eigen::Vector3f n1{};
    Eigen::Vector3f n2{};
    Eigen::Vector3f delta{};

    auto tp1 = std::chrono::steady_clock::now();
    int cnt = 0;
    for (auto i = 0; i < input_point_normal->size(); i += 10) {
#pragma omp parallel shared(cnt, input_point_normal, map, cout, i) private( \
    data, p1, p2, n1, n2, delta) default(none) num_threads(15)
      {
#pragma omp for
        for (auto j = 0; j < input_point_normal->size(); ++j) {
          if (i == j) {
            continue;
          } else {
            p1 << input_point_normal->points[i].x,
                input_point_normal->points[i].y,
                input_point_normal->points[i].z;
            p2 << input_point_normal->points[j].x,
                input_point_normal->points[j].y,
                input_point_normal->points[j].z;
            n1 << input_point_normal->points[i].normal_x,
                input_point_normal->points[i].normal_y,
                input_point_normal->points[i].normal_z;
            n2 << input_point_normal->points[j].normal_x,
                input_point_normal->points[j].normal_y,
                input_point_normal->points[j].normal_z;
            delta = p2 - p1;  // pt-pr
            float f4 = delta.norm();
            if (f4 > dobj) {
              continue;
            }
            delta.normalize();

            float f1 = atan2(delta.cross(n1).norm(), delta.dot(n1));

            float f2 = atan2(delta.cross(n2).norm(), delta.dot(n2));

            float f3 = atan2(n1.cross(n2).norm(), n1.dot(n2));

            data.second.Or = (std::make_pair(
                n1.cross(delta) / (n1.cross(delta)).norm(),
                std::make_pair(n1.cross(n1.cross(delta)) /
                                   (n1.cross(n1.cross(delta))).norm(),
                               n1 / n1.norm())));

            data.second.Ot = (std::make_pair(
                n2.cross(delta) / (n2.cross(delta)).norm(),
                std::make_pair(n2.cross(n2.cross(delta)) /
                                   (n2.cross(n2.cross(delta))).norm(),
                               n2 / n2.norm())));

            data.first.k1 =
                static_cast<int>(std::floor(f1 / angle_discretization_step));
            data.first.k2 =
                static_cast<int>(std::floor(f2 / angle_discretization_step));
            data.first.k3 =
                static_cast<int>(std::floor(f3 / angle_discretization_step));
            data.first.k4 =
                static_cast<int>(std::floor(f4 / distance_discretization_step));

            data.second.r = input_point_normal->points[i];
            data.second.t = input_point_normal->points[j];

#pragma omp critical
            map[data.first.k4][data.first.k1][data.first.k2][data.first.k3]
                .push_back(data.second);

#pragma omp critical
            cnt++;
          }
        }
      }
    }
#pragma omp barrier
    auto tp2 = std::chrono::steady_clock::now();
    std::cout << "need "
              << std::chrono::duration_cast<std::chrono::milliseconds>(tp2 -
                                                                       tp1)
                     .count()
              << "ms to process PPF" << std::endl;
    std::cout << "model中共建立" << cnt << "对PPF特征" << std::endl;
    return map;
  }

  void setDiscretizationSteps(const float &angle_discretization_step,
                              const float &distance_discretization_step);
  void setDobj(const float &dobj_) { this->dobj = dobj_; }
  PPFEstimation &operator=(const PPFEstimation &) = delete;
  PPFEstimation(const PPFEstimation &) = delete;

 private:
  float angle_discretization_step;
  float distance_discretization_step;
  float dobj;
};

#endif  // CENTRAL_VOTING_PPFESTIMATION_H
