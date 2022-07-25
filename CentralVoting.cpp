//
// Created by yyh on 22-7-11.
//
#include "CentralVoting.h"
#include "PPFEstimation.h"
#include "SmartDownSample.h"
#include "time.h"
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
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud) const {
  pcl::PointXYZ max_point, min_point;
  GenerateBound(input_cloud, max_point, min_point);
  SmartDownSample sample_filter(input_cloud,
                                std::make_pair(min_point.x, max_point.x),
                                std::make_pair(min_point.y, max_point.y),
                                std::make_pair(min_point.z, max_point.z),
                                this->step, this->AngleThreshold, 0.01);
  sample_filter.setRadius(this->normalEstimationRadius);
  return sample_filter.compute();
}

void CentralVoting::Solve() {
  std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> cloud_models_with_normal;
  clock_t start,end;
  for (auto i = 0; i < this->model_set.size(); i++) {
    auto model_cloud = SimpleDownSample(model_set[i]);
        pcl::PointCloud<pcl::PointNormal>::Ptr model_with_normal =
        DownSample(model_cloud);
    cloud_models_with_normal.push_back(model_with_normal);

    PCL_INFO("begin to establish ppf\n");
    pcl::PointCloud<pcl::PPFSignature>::Ptr cloud_model_ppf(
        new pcl::PointCloud<pcl::PPFSignature>());
    /*pcl::PPFEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PPFSignature>
        ppf_estimator;
    ppf_estimator.setInputCloud(model_with_normal);
    ppf_estimator.setInputNormals(model_with_normal);
    ppf_estimator.compute(*cloud_model_ppf);
     */

    Hash::Ptr hash_map = boost::make_shared<Hash::HashMap>();
    PPFEstimation ppf_estimator;
    ppf_estimator.setDiscretizationSteps(12.0f / 180.0f * float(M_PI), 0.05f);
    //start = clock();
    ppf_estimator.compute(model_with_normal, cloud_model_ppf, hash_map);
    //end = clock();

  }
  //std::cout<<"time:"<<end-start<<std::endl;
  PCL_INFO("finish ppf establish\n");
}

void CentralVoting::test() {
  auto model_cloud = SimpleDownSample(model_set[0]);
  pcl::PointCloud<pcl::PointNormal>::Ptr model_with_normal =
      DownSample(model_cloud);
  pcl::visualization::PCLVisualizer view("subsampled point cloud");
  view.setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> red(
      model_with_normal, 255, 0, 0);
  view.addPointCloud(model_with_normal, red, "cloud");
  view.addPointCloudNormals<pcl::PointNormal>(model_with_normal, 10, 0.5,
                                              "cloud with normal");
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

void CentralVoting::setAngleThreshold(const float &angle) {
  this->AngleThreshold = angle;
}
void CentralVoting::setDownSampleStep(const float &step) { this->step = step; }
void CentralVoting::setNormalEstimationRadius(const float &radius) {
  this->normalEstimationRadius = radius;
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
void CentralVoting::setSimpleDownSampleLeaf(const Eigen::Vector4f &subsampling_leaf_size) {
  this->subsampling_leaf_size = subsampling_leaf_size;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CentralVoting::SimpleDownSample( pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud) {
  std::cout<<"input_cloud_size:"<<input_cloud->points.size()<<std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_subsampled(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ>subsampling_filter;
  subsampling_filter.setInputCloud(input_cloud);
  subsampling_filter.setLeafSize(this->subsampling_leaf_size);
  subsampling_filter.filter(*cloud_subsampled);
  std::cout<<"output_cloud_size:"<<cloud_subsampled->points.size()<<std::endl;
  return cloud_subsampled;
}