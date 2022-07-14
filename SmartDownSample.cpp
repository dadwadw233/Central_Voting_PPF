//
// Created by yyh on 22-7-12.
//

#include "SmartDownSample.h"
#include <omp.h>
pcl::PointCloud<pcl::PointNormal>::Ptr SmartDownSample::compute() {


  PCL_INFO("\nstart to compute\n");
  PCL_INFO("\nstart to calculate normal\n");
  pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>());


  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud(this->input_cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(
      new pcl::search::KdTree<
          pcl::PointXYZ>);  ////建立kdtree来进行近邻点集搜索
  normal_estimation_filter.setSearchMethod(search_tree);
  normal_estimation_filter.setRadiusSearch(normal_estimation_search_radius);
  normal_estimation_filter.compute(*normal);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
      new pcl::PointCloud<pcl::PointNormal>());
  concatenateFields(*this->input_cloud, *normal,
                    *cloud_with_normals);
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

#pragma omp parallel shared(map) default (none)
  {
#pragma omp for
    for(int i = 0;i<map.size();i++){
#pragma omp critical
      map[i].reset(new pcl::PointCloud<pcl::PointNormal>());
    }
  }

  #pragma omp barrier
  PCL_INFO("\nfinish map init\n");

#pragma omp parallel for shared(map,x_num,y_num,cloud_with_normals) default(none) num_threads(15)
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

    //std::cout << index << std::endl;
    // store the index;
#pragma omp critical
    map[index]->points.push_back(cloud_with_normals->points[i]);
  }

#pragma omp barrier

int sum = 0;
  for(int i = 0;i<map.size();i++){
    if(!map[i]->points.empty()){
      sum+=map[i]->points.size();
    }
  }
std::cout<<sum<<std::endl;
std::cout<<this->input_cloud->points.size();//24325


  PCL_INFO("\nfinish store point\n");


pcl::PointCloud<pcl::PointXYZ>::Ptr down_sampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//#pragma omp parallel for shared(map,down_sampled_cloud) default(none) num_threads(15)
  for ( int i = 0; i < map.size(); i++) {


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