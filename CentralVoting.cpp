//
// Created by yyh on 22-7-11.
//
#include "CentralVoting.h"
#include "PPFEstimation.h"
#include "PPFRegistration.h"
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
  // pcl::visualization::PCLVisualizer view("model with center point");

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
  // std::cout<<" center: "<<center<<std::endl;
  pcl::PointXYZ p_faux(center[0], center[1], center[2]);
  pcl::PointXYZ p_saux(center[0], center[1], center[2]);
  pcl::PointXYZ c(center[0], center[1], center[2]);
  // std::vector<pcl::PointXYZ> triple;
  double d_obj = std::sqrt(std::pow(max_point_AABB.x - min_point_AABB.x, 2) +
                           std::pow(max_point_AABB.y - min_point_AABB.y, 2) +
                           std::pow(max_point_AABB.z - min_point_AABB.z, 2));
  // std::cout<<" d_obj: "<<d_obj<<std::endl;
  this->d_obj_set.push_back(static_cast<float>(d_obj));
  std::cout << "\nd_obj: " << d_obj << std::endl;
  p_faux.x -= static_cast<float>(d_obj);
  p_saux.y -= static_cast<float>(d_obj);

  this->triple_set[index].push_back(c);
  this->triple_set[index].push_back(p_faux);
  this->triple_set[index].push_back(p_saux);
  /*
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
    }*/
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
  sample_filter.setIsdense(true);
  //sample_filter.setRadius(this->normalEstimationRadius);
  sample_filter.setKSearch(this->k_point);
  return sample_filter.compute();
}

void CentralVoting::Solve() {
  /*pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud =
  boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if(this->isAdaptiveDownSample){
    scene_cloud = adaptiveDownSample(scene);
  }else{
    scene_cloud = SimpleDownSample(scene);
  }*/
  //this->scene_subsampled = DownSample(scene_cloud);
  //this->scene_subsampled = subsampleAndCalculateNormals(scene);
  Eigen::Vector4f center;
  pcl::compute3DCentroid(*scene, center);
  //this->scene_subsampled = subsampleAndCalculateNormals(scene, center[0]+200, center[1], center[2], false);
  this->scene_subsampled = subsampleAndCalculateNormals(scene, Eigen::Vector4f(28.0f, 28.0f, 28.0f, 0.0f));
  // pcl::copyPointCloud(*scene, *this->scene_subsampled);
  std::cout<<center<<std::endl;
  std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> cloud_models_with_normal;
  std::vector<Hash::Ptr> hashmap_search_vector;
  for (auto i = 0; i < this->model_set.size(); i++) {
    //auto model_cloud = SimpleDownSample(model_set[i]);
    //pcl::PointCloud<pcl::PointNormal>::Ptr model_with_normal =
        //DownSample(model_cloud);
    // pcl::PointCloud<pcl::PointNormal>::Ptr model_with_normal =
    // subsampleAndCalculateNormals(model_set[i]);
    pcl::PointCloud<pcl::PointNormal>::Ptr model_with_normal =
        subsampleAndCalculateNormals(model_set[i], this->triple_set[i], true);
    cloud_models_with_normal.push_back(model_with_normal);
/**
 * 可视化法线
 *
 *
 *

    pcl::visualization::PCLVisualizer view("subsampled point cloud");
    view.setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> red(
        model_with_normal, 255, 0, 0);
    view.addPointCloud(model_with_normal, red, "model");
    view.addPointCloudNormals<pcl::PointNormal>(model_with_normal, 1, 5,
                                                "model with normal");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> white(
        scene_subsampled, 0, 255, 0);
    view.addPointCloud(scene_subsampled, white, "scene");
    view.addPointCloudNormals<pcl::PointNormal>(scene_subsampled, 1, 5, "scene with normals");
    while (!view.wasStopped()) {
      view.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }
   **/
    PCL_INFO("begin to establish ppf\n");
    pcl::PointCloud<pcl::PPFSignature>::Ptr cloud_model_ppf(
        new pcl::PointCloud<pcl::PPFSignature>());

    Hash::Ptr hash_map = boost::make_shared<Hash::HashMap>();
    PPFEstimation ppf_estimator;
    ppf_estimator.setDiscretizationSteps(12.0f / 180.0f * float(M_PI), 0.05f);
    // start = clock();
    ppf_estimator.compute(model_with_normal, cloud_model_ppf, hash_map);

    hashmap_search_vector.push_back(hash_map);
  }
  // std::cout<<"time:"<<end-start<<std::endl;
  PCL_INFO("finish ppf establish\n");

  PCL_INFO("Registering models to scene ...\n");

  pcl::visualization::PCLVisualizer view("registration result");
  view.setBackgroundColor(0, 0, 0);
  auto tp1 = std::chrono::steady_clock::now();
  for (std::size_t model_i = 0; model_i < model_set.size(); ++model_i) {
    PPFRegistration ppf_registration{};
    ppf_registration.setSceneReferencePointSamplingRate(10);
    ppf_registration.setPositionClusteringThreshold(3);//投票的体素网格的size
    ppf_registration.setRotationClusteringThreshold(30.0f / 180.0f *
                                                    float(M_PI));
    ppf_registration.setSearchMap(hashmap_search_vector[model_i]);
    ppf_registration.setInputSource(cloud_models_with_normal[model_i]);
    ppf_registration.setInputTarget(this->scene_subsampled);
    ppf_registration.setModelTripleSet(this->triple_set[model_i]);
    ppf_registration.setDobj(this->d_obj_set[model_i]);
    ppf_registration.setDiscretizationSteps(12.0f / 180.0f * float(M_PI),
                                            0.05f);
    ppf_registration.compute();
    Eigen::Affine3f T = ppf_registration.getFinalTransformation();
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_model(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*this->model_set[model_i], *output_model, T);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(
        output_model, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white(
        this->scene, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> s(
        this->model_set[model_i], 0, 255, 0);
    // view.addPointCloud(model_set[model_i], s, "model");
    view.addPointCloud(output_model, red, "out");
    view.addPointCloud(this->scene, white, "scene");
  }
  auto tp2 = std::chrono::steady_clock::now();
  std::cout << "\nneed "
            << std::chrono::duration_cast<std::chrono::milliseconds>(tp2 - tp1)
                   .count()
            << "ms for online process" << std::endl;
  while (!view.wasStopped()) {
    view.spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  }
}

void CentralVoting::test() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_ =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (isAdaptiveDownSample) {
    scene_ = adaptiveDownSample(this->scene);
  } else {
    scene_ = SimpleDownSample(this->scene);
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr model_with_normal =
      DownSample(scene_);
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
    this->InitTripleSet();
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
void CentralVoting::setSimpleDownSampleLeaf(
    const Eigen::Vector4f &subsampling_leaf_size) {
  this->subsampling_leaf_size = subsampling_leaf_size;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CentralVoting::SimpleDownSample(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud) {
  std::cout << "input_cloud_size:" << input_cloud->points.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_subsampled(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> subsampling_filter;
  subsampling_filter.setInputCloud(input_cloud);
  subsampling_filter.setLeafSize(this->subsampling_leaf_size);
  subsampling_filter.filter(*cloud_subsampled);
  std::cout << "output_cloud_size:" << cloud_subsampled->points.size()
            << std::endl;
  return cloud_subsampled;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CentralVoting::adaptiveDownSample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
  std::cout << "input_cloud_size:" << input_cloud->points.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_subsampled(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> subsampling_filter;
  subsampling_filter.setInputCloud(input_cloud);
  Eigen::Vector4f leaf_size;

  if (this->adaptive_step != 0) {
    leaf_size << adaptive_step, adaptive_step, adaptive_step, 0.0f;
  } else {
    pcl::PointXYZ min_p, max_p;
    GenerateBound(input_cloud, max_p, min_p);
    float max_l = std::abs(static_cast<float>(std::max(
        max_p.x - min_p.x, std::max(max_p.y - min_p.y, max_p.z - min_p.z))));
    float s = std::ceil(max_l / pow(this->downSampleTarget, (0.5)));
    std::cout << "max_l: " << max_l << std::endl;
    std::cout << "adaptive step: " << s << std::endl;
    leaf_size << s, s, s, 0.0f;
  }

  subsampling_filter.setLeafSize(leaf_size);
  subsampling_filter.filter(*cloud_subsampled);
  std::cout << "output_cloud_size:" << cloud_subsampled->points.size()
            << std::endl;
  return cloud_subsampled;
}

void CentralVoting::setAdaptiveDownSampleOption(const bool &lhs, const int &rhs,
                                                const float &step_) {
  this->isAdaptiveDownSample = lhs;
  this->downSampleTarget = rhs;
  if (step_ != 0) {
    this->adaptive_step = step_;
  }
}


pcl::PointCloud<pcl::PointNormal>::Ptr
CentralVoting::subsampleAndCalculateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)  //降采样并计算表面法向量
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_subsampled(
      new pcl::PointCloud<
          pcl::PointXYZ>());  //直接进行降采样，没有进行额外的处理
  pcl::VoxelGrid<pcl::PointXYZ> subsampling_filter;  //创建体素栅格
  subsampling_filter.setInputCloud(cloud);
  subsampling_filter.setLeafSize(subsampling_leaf_size);  // 设置采样体素大小
  subsampling_filter.filter(*cloud_subsampled);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_subsampled_normals(
      new pcl::PointCloud<pcl::Normal>());
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud(cloud_subsampled);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(
      new pcl::search::KdTree<pcl::PointXYZ>);  ////建立kdtree来进行近邻点集搜索
  normal_estimation_filter.setSearchMethod(search_tree);
  normal_estimation_filter.setKSearch(k_point);
  //normal_estimation_filter.setRadiusSearch(normalEstimationRadius);
  normal_estimation_filter.compute(*cloud_subsampled_normals);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_subsampled_with_normals(
      new pcl::PointCloud<pcl::PointNormal>());
  concatenateFields(
      *cloud_subsampled, *cloud_subsampled_normals,
      *cloud_subsampled_with_normals);  // concatenate point cloud and its
                                        // normal into a new cloud

  PCL_INFO("Cloud dimensions before / after subsampling: %zu / %zu\n",
           static_cast<std::size_t>(cloud->size()),
           static_cast<std::size_t>(cloud_subsampled->size()));
  return cloud_subsampled_with_normals;
}

pcl::PointCloud<pcl::PointNormal>::Ptr
CentralVoting::subsampleAndCalculateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const Eigen::Vector4f &leaf_size) const  //降采样并计算表面法向量
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_subsampled(
      new pcl::PointCloud<
          pcl::PointXYZ>());  //直接进行降采样，没有进行额外的处理
  pcl::VoxelGrid<pcl::PointXYZ> subsampling_filter;  //创建体素栅格
  subsampling_filter.setInputCloud(cloud);
  subsampling_filter.setLeafSize(leaf_size);  // 设置采样体素大小
  subsampling_filter.filter(*cloud_subsampled);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_subsampled_normals(
      new pcl::PointCloud<pcl::Normal>());
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud(cloud_subsampled);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(
      new pcl::search::KdTree<pcl::PointXYZ>);  ////建立kdtree来进行近邻点集搜索
  normal_estimation_filter.setSearchMethod(search_tree);
  normal_estimation_filter.setKSearch(k_point);
  //normal_estimation_filter.setRadiusSearch(normalEstimationRadius);
  normal_estimation_filter.compute(*cloud_subsampled_normals);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_subsampled_with_normals(
      new pcl::PointCloud<pcl::PointNormal>());
  concatenateFields(
      *cloud_subsampled, *cloud_subsampled_normals,
      *cloud_subsampled_with_normals);  // concatenate point cloud and its
                                        // normal into a new cloud

  PCL_INFO("Cloud dimensions before / after subsampling: %zu / %zu\n",
           static_cast<std::size_t>(cloud->size()),
           static_cast<std::size_t>(cloud_subsampled->size()));
  return cloud_subsampled_with_normals;
}

pcl::PointCloud<pcl::PointNormal>::Ptr
CentralVoting::subsampleAndCalculateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::vector<pcl::PointXYZ> &view_point, const bool &reverse) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_subsampled(
      new pcl::PointCloud<
          pcl::PointXYZ>());  //直接进行降采样，没有进行额外的处理
  pcl::VoxelGrid<pcl::PointXYZ> subsampling_filter;  //创建体素栅格
  subsampling_filter.setInputCloud(cloud);
  subsampling_filter.setLeafSize(subsampling_leaf_size);  // 设置采样体素大小
  subsampling_filter.filter(*cloud_subsampled);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_subsampled_normals(
      new pcl::PointCloud<pcl::Normal>());
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation_filter;
  normal_estimation_filter.setViewPoint(view_point[0].x, view_point[0].y,
                                        view_point[0].z);
  normal_estimation_filter.setInputCloud(cloud_subsampled);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(
      new pcl::search::KdTree<pcl::PointXYZ>);  ////建立kdtree来进行近邻点集搜索
  normal_estimation_filter.setSearchMethod(search_tree);
  normal_estimation_filter.setKSearch(k_point);
  //normal_estimation_filter.setRadiusSearch(normalEstimationRadius);
  normal_estimation_filter.compute(*cloud_subsampled_normals);
  if (reverse) {
    for (auto i : *cloud_subsampled_normals) {
      i.normal_x = -i.normal_x;
      i.normal_y = -i.normal_y;
      i.normal_z = -i.normal_z;
      i.normal[0] = i.normal_x;
      i.normal[1] = i.normal_y;
      i.normal[2] = i.normal_z;
      i.curvature = -i.curvature;
    }
  }
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_subsampled_with_normals(
      new pcl::PointCloud<pcl::PointNormal>());
  concatenateFields(
      *cloud_subsampled, *cloud_subsampled_normals,
      *cloud_subsampled_with_normals);  // concatenate point cloud and its
                                        // normal into a new cloud

  PCL_INFO("Cloud dimensions before / after subsampling: %zu / %zu\n",
           static_cast<std::size_t>(cloud->size()),
           static_cast<std::size_t>(cloud_subsampled->size()));
  return cloud_subsampled_with_normals;
}

pcl::PointCloud<pcl::PointNormal>::Ptr
CentralVoting::subsampleAndCalculateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const float x,
    const float y, const float z, const bool &reverse) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_subsampled(
      new pcl::PointCloud<
          pcl::PointXYZ>());  //直接进行降采样，没有进行额外的处理
  pcl::VoxelGrid<pcl::PointXYZ> subsampling_filter;  //创建体素栅格
  subsampling_filter.setInputCloud(cloud);
  subsampling_filter.setLeafSize(subsampling_leaf_size);  // 设置采样体素大小
  subsampling_filter.filter(*cloud_subsampled);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_subsampled_normals(
      new pcl::PointCloud<pcl::Normal>());
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation_filter;
  normal_estimation_filter.setViewPoint(x, y, z);
  normal_estimation_filter.setInputCloud(cloud_subsampled);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(
      new pcl::search::KdTree<pcl::PointXYZ>);  ////建立kdtree来进行近邻点集搜索
  normal_estimation_filter.setSearchMethod(search_tree);
  normal_estimation_filter.setKSearch(k_point);
  //normal_estimation_filter.setRadiusSearch(normalEstimationRadius);
  normal_estimation_filter.compute(*cloud_subsampled_normals);
  if (reverse) {
    for (auto i : *cloud_subsampled_normals) {
      i.normal_x = -i.normal_x;
      i.normal_y = -i.normal_y;
      i.normal_z = -i.normal_z;
      i.normal[0] = i.normal_x;
      i.normal[1] = i.normal_y;
      i.normal[2] = i.normal_z;
      i.curvature = -i.curvature;
    }
  }
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_subsampled_with_normals(
      new pcl::PointCloud<pcl::PointNormal>());
  concatenateFields(
      *cloud_subsampled, *cloud_subsampled_normals,
      *cloud_subsampled_with_normals);  // concatenate point cloud and its
                                        // normal into a new cloud

  PCL_INFO("Cloud dimensions before / after subsampling: %zu / %zu\n",
           static_cast<std::size_t>(cloud->size()),
           static_cast<std::size_t>(cloud_subsampled->size()));
  return cloud_subsampled_with_normals;
}