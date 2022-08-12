//
// Created by yyh on 22-7-12.
//

#include "SmartDownSample.h"
#include <omp.h>
pcl::PointCloud<pcl::PointNormal>::Ptr SmartDownSample::compute() {
  std::cout << "before down sample" << this->input_cloud->points.size();
  PCL_INFO("\nstart to compute\n");
  PCL_INFO("\nstart to calculate normal\n");
  pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>());

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud(this->input_cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(
      new pcl::search::KdTree<pcl::PointXYZ>);  ////建立kdtree来进行近邻点集搜索
  normal_estimation_filter.setSearchMethod(search_tree);
  if(isSetRadius){
    normal_estimation_filter.setRadiusSearch(normal_estimation_search_radius);
  }
  else{
    normal_estimation_filter.setKSearch(normal_estimation_search_k_points);
  }
  normal_estimation_filter.compute(*normal);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
      new pcl::PointCloud<pcl::PointNormal>());
  concatenateFields(*this->input_cloud, *normal, *cloud_with_normals);
#pragma omp barrier
  PCL_INFO("\nfinish normal calculation\n");
  pcl::PointCloud<pcl::PointNormal>::Ptr output_cloud(
      new pcl::PointCloud<pcl::PointNormal>());

  std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>
      map;  // define the voxel grid , store the index of the point in cloud
  PCL_INFO("\nstart map init\n");

  auto xr =
      std::abs(static_cast<float>(this->x_range.second - this->x_range.first));
  auto yr =
      std::abs(static_cast<float>(this->y_range.second - this->y_range.first));
  auto zr =
      std::abs(static_cast<float>(this->z_range.second - this->z_range.first));

  PCL_INFO("\nhalf\n");
  auto x_num = static_cast<long long int>(std::ceil(xr / this->step));
  auto y_num = static_cast<long long int>(std::ceil(yr / this->step));
  auto z_num = static_cast<long long int>(std::ceil(zr / this->step));

  map.resize(x_num * y_num * z_num);

#pragma omp parallel shared(map) default(none)
  {
#pragma omp for
    for (int i = 0; i < map.size(); i++) {
#pragma omp critical
      map[i].reset(new pcl::PointCloud<pcl::PointNormal>());
    }
  }

#pragma omp barrier
  PCL_INFO("\nfinish map init\n");

#pragma omp parallel for shared(map, x_num, y_num, cloud_with_normals, \
                                cout) default(none) num_threads(15)
  for (int i = 0; i < this->input_cloud->points.size(); i++) {
    const int xCell =
        static_cast<int>(std::ceil(
            (input_cloud->points[i].x - this->x_range.first) / step)) == 0
            ? 1
            : static_cast<int>(std::ceil(
                  (input_cloud->points[i].x - this->x_range.first) / step));
    const int yCell =
        static_cast<int>(std::ceil(
            (input_cloud->points[i].y - this->y_range.first) / step)) == 0
            ? 1
            : static_cast<int>(std::ceil(
                  (input_cloud->points[i].y - this->y_range.first) / step));
    const int zCell =
        static_cast<int>(std::ceil(
            (input_cloud->points[i].z - this->z_range.first) / step)) == 0
            ? 1
            : static_cast<int>(std::ceil(
                  (input_cloud->points[i].z - this->z_range.first) / step));

    const int index =
        (xCell - 1) + (yCell - 1) * x_num + (zCell - 1) * x_num * y_num;

    // std::cout << index << std::endl;
    // store the index;
#pragma omp critical
    map[index]->points.push_back(cloud_with_normals->points[i]);

    // std::cout<<cloud_with_normals->points[i].normal_x<<std::endl;
  }

#pragma omp barrier

  int sum = 0;
  for (int i = 0; i < map.size(); i++) {
    if (!map[i]->points.empty()) {
      sum += map[i]->points.size();
    }
  }
  std::cout << sum << std::endl;
  std::cout << this->input_cloud->points.size();  // 24325

  PCL_INFO("\nfinish store point\n");

#pragma omp parallel for shared(map, output_cloud, cout) default(none) \
    num_threads(15)
  for (int i = 0; i < map.size(); i++) {
    int cnt = 0;
    if (map[i]->points.empty()) {
      continue;
    } else if (map[i]->points.size() == 1 && isdense) {
#pragma omp critical
      output_cloud->points.push_back(map[i]->points[0]);
      continue;
    } else {
      bool flag[map[i]->points.size()];
      for (int p = 0; p < map[i]->points.size(); p++) {
        flag[p] = false;
      }
      for (int j = 0; j < map[i]->points.size(); j++) {
        if (cnt == map[i]->points.size() ||
            (cnt >= map[i]->points.size() / 3 && !isdense)) {
          break;
        }

        for (int k = 0; k < map[i]->points.size(); k++) {
          if (j == k) {
            continue;
          } else if (flag[j] && flag[k]) {
            continue;
          } else {
            if (pcl::getAngle3D(static_cast<const Eigen::Vector3f>(
                                    map[i]->points[j].normal),
                                static_cast<const Eigen::Vector3f>(
                                    map[i]->points[k].normal),
                                true) >= this->angleThreshold) {
              if (!flag[j]) {
                flag[j] = true;
#pragma omp critical
                output_cloud->points.push_back(map[i]->points[j]);
                cnt++;
              }
              if (!flag[k]) {
                flag[k] = true;
#pragma omp critical
                output_cloud->points.push_back(map[i]->points[k]);
                cnt++;
              }
            }
          }
        }
      }
    }
    if (cnt == 0 && isdense) {
      Eigen::Vector4f center;
      pcl::compute3DCentroid(*map[i], center);
      pcl::PointNormal p;
      p.x = center(0);
      p.y = center(1);
      p.z = center(2);
      map[i]->points.push_back(p);
      //法向量求解器
      pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal>
          pointNormalEstimation;
      //法向量
      pcl::PointNormal pointNormal;
      //我们要求解的点,这个点的index你可以自己设置
      pcl::PointNormal searchPoint = map[i]->points[map[i]->points.size() - 1];
      // KNN
      pcl::search::KdTree<pcl::PointNormal>::Ptr tree(
          new pcl::search::KdTree<pcl::PointNormal>());
      //设置KdTree要求解的点云参数
      tree->setInputCloud(map[i]);
      //这是K近邻的半径
      float radius = this->normal_estimation_search_radius;
      //关键的数据indices
      std::vector<int> indices;
      //每个点到searchPoint的距离(暂时用不到)
      std::vector<float> distance;

      if(isSetPoints){
        tree->nearestKSearch(searchPoint, map.size()-1, indices, distance);
      }else{
        tree->radiusSearch(searchPoint, radius, indices, distance);
      }

      //输出参数1>平面数据(可以转化为法向量)
      Eigen::Vector4f planeParams;
      //输出参数2>平面曲率
      float curvature;
      //进行单个点的法向量求解
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::copyPointCloud(*map[i], cloud);
      pointNormalEstimation.computePointNormal(cloud, indices, planeParams,
                                               curvature);
      pointNormal.x = searchPoint.x;
      pointNormal.y = searchPoint.y;
      pointNormal.z = searchPoint.z;
      pointNormal.normal_x = planeParams[0];
      pointNormal.normal_y = planeParams[1];
      pointNormal.normal_z = planeParams[2];
      pointNormal.curvature = curvature;
#pragma omp critical
      output_cloud->points.push_back(pointNormal);
    }
  }
  PCL_INFO("\ndown sample finish\n");
  std::cout << "after down sample" << output_cloud->points.size() << std::endl;
  return output_cloud;
}
void SmartDownSample::setIsdense(const bool &data) { this->isdense = data; }
void SmartDownSample::setRadius(float data) {
  this->normal_estimation_search_radius = data;
  this->isSetPoints = false;
  this->isSetRadius = true;
}
template <class T>
float SmartDownSample::calculateDistance(T &pointA, T &pointB) {
  float distance = std::pow((pointA.x - pointB.x), 2) +
                   std::pow((pointA.y - pointB.y), 2) +
                   std::pow((pointA.z - pointB.z), 2);
  return distance;
}
template <class T>
float SmartDownSample::calculateDistance(T &pointA, pcl::PointNormal &pointB) {
  float distance = std::pow((pointA.x - pointB.x), 2) +
                   std::pow((pointA.y - pointB.y), 2) +
                   std::pow((pointA.z - pointB.z), 2);
  return distance;
}

void SmartDownSample::setKSearch(const int data) {
  this->normal_estimation_search_k_points = data;
  this->isSetPoints = true;
  this->isSetRadius = false;
}