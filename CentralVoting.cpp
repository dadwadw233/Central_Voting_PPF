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
    pcl::visualization::PCLVisualizer view("model with center point");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_color(
        255, 255, 255);
    view.addPointCloud(this->model_set[index], model_color, "model");

    view.setBackgroundColor(0, 0, 0);

    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(this->model_set[index]);
    feature_extractor.compute();

    std::vector<float> moment_of_inertia;
    std::vector<float> eccentricity;
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    feature_extractor.getEigenVectors(major_vector, middle_vector,
                                      minor_vector);
    feature_extractor.getMassCenter(mass_center);
    view.addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y,
                 max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0,
                 0.0, "AABB");
    double d_obj = std::sqrt(std::pow(max_point_AABB.x - min_point_AABB.x, 2) +
                             std::pow(max_point_AABB.y - min_point_AABB.y, 2) +
                             std::pow(max_point_AABB.z - min_point_AABB.z, 2));

    view.setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
    pcl::PointXYZ center_(mass_center(0), mass_center(1), mass_center(2));
    pcl::PointXYZ x_axis(major_vector(0) * 100 + mass_center(0),
                         major_vector(1) * 100 + mass_center(1),
                         major_vector(2) * 100 + mass_center(2));
    pcl::PointXYZ y_axis(middle_vector(0) * 100 + mass_center(0),
                         middle_vector(1) * 100 + mass_center(1),
                         middle_vector(2) * 100 + mass_center(2));
    pcl::PointXYZ z_axis(minor_vector(0) * 100 + mass_center(0),
                         minor_vector(1) * 100 + mass_center(1),
                         minor_vector(2) * 100 + mass_center(2));
    view.addLine(center_, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
    view.addLine(center_, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    view.addLine(center_, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

    pcl::PointXYZ p_faux(center_);
    pcl::PointXYZ p_saux(center_);
    std::vector<pcl::PointXYZ> triple;
    p_faux.x -= static_cast<float>(d_obj);
    p_saux.y -= static_cast<float>(d_obj);
    this->InitTripleSet();
    this->triple_set[index].push_back(center_);
    this->triple_set[index].push_back(p_faux);
    this->triple_set[index].push_back(p_saux);

    pcl::PointCloud<pcl::PointXYZ>::Ptr triple_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    triple_cloud->points.push_back(center_);
    triple_cloud->points.push_back(p_faux);
    triple_cloud->points.push_back(p_saux);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(
        triple_cloud, 255, 0, 0);
    view.addPointCloud(triple_cloud, red, "triple");
    view.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "triple");

    while (!view.wasStopped()) {
      view.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }
  }
}
void CentralVoting::InitTripleSet() {
  this->triple_set.resize(this->model_set.size());
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
