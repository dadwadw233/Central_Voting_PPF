//
// Created by yyh on 22-7-12.
//

#include "SmartDownSample.h"
#include <omp.h>
pcl::PointCloud<pcl::PointNormal>::Ptr SmartDownSample::compute() {
  PCL_INFO("\nstart to compute\n");
  pcl::PointCloud<pcl::PointNormal>::Ptr output_cloud(
      new pcl::PointCloud<pcl::PointNormal>());

  std::vector<std::vector<int>>
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

  PCL_INFO("\nfinish map init\n");
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
    map[index].push_back(i);
  }
  PCL_INFO("finish store point");

  int i,j,k;
//#pragma omp parallel private(i,j,k)
  for ( i = 0; i < map.size(); i++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(
        new pcl::PointCloud<pcl::PointXYZ>());
    for ( j = 0; j < map[i].size(); j++) {
      temp->points.push_back(this->input_cloud->points[map[i][j]]);
    }
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(temp);
    feature_extractor.compute();
    Eigen::Vector3f mass_center;
    feature_extractor.getMassCenter(mass_center);
    pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
    temp->points.push_back(center);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_subsampled_normals(
        new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation_filter;
    normal_estimation_filter.setInputCloud(temp);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(
        new pcl::search::KdTree<
            pcl::PointXYZ>);  ////建立kdtree来进行近邻点集搜索
    normal_estimation_filter.setSearchMethod(search_tree);
    normal_estimation_filter.setRadiusSearch(normal_estimation_search_radius);
    normal_estimation_filter.compute(*cloud_subsampled_normals);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_subsampled_with_normals(
        new pcl::PointCloud<pcl::PointNormal>());
    concatenateFields(*temp, *cloud_subsampled_normals,
                      *cloud_subsampled_with_normals);
    for ( k = 0; k < cloud_subsampled_with_normals->points.size(); k++) {
      if (calculateDistance(center, cloud_subsampled_with_normals->points[k]) <
          distanceThreshold) {
      }
    }
  }
  return output_cloud;
}



void SmartDownSample::setRadius(float data) {
  this->normal_estimation_search_radius = data;
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