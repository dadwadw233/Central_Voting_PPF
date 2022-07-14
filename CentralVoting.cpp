//
// Created by yyh on 22-7-11.
//
#include "CentralVoting.h"
#include "SmartDownSample.h"
void CentralVoting::CenterExtractor(int index) {
  Eigen::Vector4f center;
  pcl::compute3DCentroid(*this->model_set[index], center);
  std::cout << "pcl函数计算质心结果" << std::endl << center;
  pcl::PointXYZ p;
  p.x = center(0);
  p.y = center(1);
  p.z = center(2);

  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(this->model_set[index]);
  feature_extractor.compute();

  pcl::PointXYZ min_point_AABB;
  pcl::PointXYZ max_point_AABB;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;
  feature_extractor.getAABB(min_point_AABB, max_point_AABB);
  feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter(mass_center);
  std::cout << "min_point:" << min_point_AABB << std::endl
            << "max_point:" << max_point_AABB << std::endl;
  pcl::visualization::PCLVisualizer view("model with center point");

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
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_color(
      255, 255, 255);

  pcl::PointXYZ p_faux(center_);
  pcl::PointXYZ p_saux(center_);
  std::vector<pcl::PointXYZ> triple;
  double d_obj = std::sqrt(std::pow(max_point_AABB.x - min_point_AABB.x, 2) +
                           std::pow(max_point_AABB.y - min_point_AABB.y, 2) +
                           std::pow(max_point_AABB.z - min_point_AABB.z, 2));
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

  // visualize
  view.addPointCloud(this->model_set[index], model_color, "model");

  view.setBackgroundColor(0, 0, 0);

  view.addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y,
               max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0,
               0.0, "AABB");
  view.setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
  view.addLine(center_, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
  view.addLine(center_, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
  view.addLine(center_, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
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

pcl::PointCloud<pcl::PointNormal>::Ptr CentralVoting::DownSample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud) {
  pcl::PointXYZ max_point, min_point;
  GenerateBound(input_cloud, max_point, min_point);
  SmartDownSample sample_filter(
      input_cloud, std::make_pair(min_point.x, max_point.x),
      std::make_pair(min_point.y, max_point.y),
      std::make_pair(min_point.z, max_point.z), 10, 30, 0.01);
  sample_filter.setRadius(10.0f);
  return sample_filter.compute();
}

void CentralVoting::EstablishPPF(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud) {
  DownSample(input_cloud);
}
void CentralVoting::test() {
  this->model_subsampled = DownSample(this->model_set[0]);
  pcl::visualization::PCLVisualizer view("subsampled point cloud");
  view.setBackgroundColor(0,0,0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> red(
      this->model_subsampled, 255, 0, 0);
  view.addPointCloud(this->model_subsampled,red,"cloud");
  view.addPointCloudNormals<pcl::PointNormal>(this->model_subsampled,10,0.5,"cloud with normal");
  while (!view.wasStopped()) {
    view.spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  }
}
void CentralVoting::GenerateBound(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
    pcl::PointXYZ &max_point, pcl::PointXYZ &min_point) {
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(input_cloud);
  feature_extractor.compute();
  feature_extractor.getAABB(min_point, max_point);
}

bool CentralVoting::CenterExtractorAll() {
  if (this->model_set.empty()) {
    PCL_ERROR("there is no model point cloud in the model set\n");
    return false;
  } else {
    for (auto i = 0; i < this->model_set.size(); i++) {
      CenterExtractor(i);
    }
    PCL_INFO("All models has finished triple set extraction\n");
    return true;
  }
}

void CentralVoting::InitTripleSet() {
  this->triple_set.resize(this->model_set.size());
}

bool CentralVoting::AddModel(pcl::PointCloud<pcl::PointXYZ>::Ptr input_model) {
  if (this->model_set.size() > maxModelNum) {
    PCL_ERROR("model vector is full");
    return false;
  } else {
    this->model_set.push_back(std::move(input_model));
    return true;
  }
}
