//
// Created by yyh on 22-7-11.
//
#include "CentralVoting.h"

bool CentralVoting::CenterExtractor(int index) {
  if (this->model_set.empty()) {
    PCL_ERROR("there is no model point cloud in the model set");
  } else {
    Eigen::Vector4f center;
    pcl::compute3DCentroid(*this->model_set[index], center);
    std::cout << "pcl函数计算质心结果" << center;
    pcl::PointXYZ p;
    p.x = center(0);
    p.y = center(1);
    p.z = center(2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>());
    c->points.push_back(p);

    pcl::visualization::PCLVisualizer view("model with center point");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(c, 255,
                                                                        0, 0);
    view.addPointCloud(c, red, "center");
    view.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "center");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_color(
        255, 255, 255);
    view.addPointCloud(this->model_set[index], model_color, "model");

    view.setBackgroundColor(0, 0, 0);

    while (!view.wasStopped()) {
      view.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }
  }
}
bool CentralVoting::AddModel(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_model) {
  if (this->model_set.size() > maxModelNum) {
    PCL_ERROR("model vector is full");
    return false;
  } else {
    this->model_set.push_back(std::move(input_model));
    return true;
  }
}
