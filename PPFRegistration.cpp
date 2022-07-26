//
// Created by yyh on 22-7-26.
//

#include "PPFRegistration.h"

PPFRegistration::PPFRegistration() {
  model_cloud_with_normal.reset( new pcl::PointCloud<pcl::PointNormal>());
  scene_cloud_with_normal.reset(new pcl::PointCloud<pcl::PointNormal>());
  searchMap.reset(new Hash::HashMap());
}
void PPFRegistration::setSceneReferencePointSamplingRate(const float &scene_reference_point_sampling_rate) {
  this->scene_reference_point_sampling_rate = scene_reference_point_sampling_rate;
}

void PPFRegistration::setPositionClusteringThreshold(const float &clustering_position_diff_threshold) {
  this->clustering_position_diff_threshold = clustering_position_diff_threshold;
}

void PPFRegistration::setRotationClusteringThreshold(const float &clustering_rotation_diff_threshold) {
  this->clustering_rotation_diff_threshold = clustering_rotation_diff_threshold;
}
void PPFRegistration::setInputTarget(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud) {
  this->model_cloud_with_normal = cloud;
}
void PPFRegistration::setInputSource(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud) {
  this->scene_cloud_with_normal = cloud;
}
void PPFRegistration::setSearchMap(const Hash::Ptr &searchMap) {
  this->searchMap = searchMap;
}
void PPFRegistration::setDiscretizationSteps(
    const float &angle_discretization_step,
    const float &distance_discretization_step) {
  this->angle_discretization_step = angle_discretization_step;
  this->distance_discretization_step = distance_discretization_step;
}

decltype(auto) PPFRegistration::getFinalTransformation() {
  return this->finalTransformation;
}
template<class T>
typename pcl::PointCloud<T>::Ptr PPFRegistration::aligen(const typename pcl::PointCloud<T>::Ptr &input) {
  typename pcl::PointCloud<T>::Ptr output = boost::make_shared<pcl::PointCloud<T>>();
    pcl::transformPointCloud(*input,*output,finalTransformation);
    return output;
}
void PPFRegistration::setModelTripleSet(const std::vector<pcl::PointXYZ> &triple_set) {
  this->triple_set.resize(triple_set.size());
  for(size_t i = 0;i<this->triple_set.size();++i){
    this->triple_set.push_back(triple_set[i]);
  }
}

void PPFRegistration::compute() {
  pcl::PPFSignature feature{};
  std::pair<Hash::HashKey, Hash::HashData> data{};
  Eigen::Vector4f p1{};
  Eigen::Vector4f p2{};
  Eigen::Vector4f n1{};
  Eigen::Vector4f n2{};
  Eigen::Vector4f delta{};
  auto tp1 = boost::chrono::steady_clock::now();

  for(auto i = 0;i<scene_cloud_with_normal->points.size();++i){
    for(auto j = 0;j<scene_cloud_with_normal->points.size();++j){
      if(i == j){
        continue;
      }else{
        p1 << scene_cloud_with_normal->points[i].x,
            scene_cloud_with_normal->points[i].y, scene_cloud_with_normal->points[i].z,
            0.0f;
        p2 << scene_cloud_with_normal->points[j].x,
            scene_cloud_with_normal->points[j].y, scene_cloud_with_normal->points[j].z,
            0.0f;
        n1 << scene_cloud_with_normal->points[i].normal_x,
            scene_cloud_with_normal->points[i].normal_y,
            scene_cloud_with_normal->points[i].normal_z, 0.0f;
        n2 << scene_cloud_with_normal->points[j].normal_x,
            scene_cloud_with_normal->points[j].normal_y,
            scene_cloud_with_normal->points[j].normal_z, 0.0f;

        delta = p2 - p1;
        float f4 = delta.norm();

        // normalize
        delta /= f4;


        float f1 = n1[0] * delta[0] + n1[1] * delta[1] + n1[2] * delta[2];

        float f2 = n1[0] * delta[0] + n2[1] * delta[1] + n2[2] * delta[2];

        float f3 = n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2];

        /*float f1 = n1.x() * delta.x() + n1.y()  * delta.y() + n2.z()  * delta.z();

        float f2 = n1.x() * delta.x() + n2.y()  * delta.y() + n2.z()  * delta.z();

        float f3 = n1.x() * n2.x() + n1.y()  * n2.y()  + n1.z()  * n2.z() ;
         */
        feature.f1 = f1;
        feature.f2 = f2;
        feature.f3 = f3;
        feature.f4 = f4;
        feature.alpha_m = 0.0f;
        data.second.Or =
            (std::make_pair(n1.cross3(delta),
                            std::make_pair(n1.cross3(n1.cross3(delta)), n1)));
        data.second.Ot =
            (std::make_pair(n2.cross3(delta),
                            std::make_pair(n2.cross3(n2.cross3(delta)), n2)));

        data.first.k1 =
            static_cast<int>(std::floor(f1 / angle_discretization_step));
        data.first.k2 =
            static_cast<int>(std::floor(f2 / angle_discretization_step));
        data.first.k3 =
            static_cast<int>(std::floor(f3 / angle_discretization_step));
        data.first.k4 =
            static_cast<int>(std::floor(f4 / distance_discretization_step));

        if(searchMap->find(data.first)){
            auto model_lrf = this->searchMap->getData(data.first);
            Eigen::Matrix3f model_lrf_Or;
            Eigen::Matrix3f model_lrf_Ot;
            Eigen::Matrix3f scene_lrf_Or;
            Eigen::Matrix3f scene_lrf_Ot;

            model_lrf_Or<<
                model_lrf.Or.first[0], model_lrf.Or.second.first[0], model_lrf.Or.second.second[0],
                model_lrf.Or.first[1], model_lrf.Or.second.first[1], model_lrf.Or.second.second[1],
                model_lrf.Or.first[2], model_lrf.Or.second.first[2], model_lrf.Or.second.second[2];
            model_lrf_Ot<<
                model_lrf.Ot.first[0], model_lrf.Ot.second.first[0], model_lrf.Ot.second.second[0],
                model_lrf.Ot.first[1], model_lrf.Ot.second.first[1], model_lrf.Ot.second.second[1],
                model_lrf.Ot.first[2], model_lrf.Ot.second.first[2], model_lrf.Ot.second.second[2];
            scene_lrf_Or<<
                data.second.Or.first[0], data.second.Or.second.first[0], data.second.Or.second.second[0],
                data.second.Or.first[1], data.second.Or.second.first[1], data.second.Or.second.second[1],
                data.second.Or.first[2], data.second.Or.second.first[2], data.second.Or.second.second[2];
            scene_lrf_Ot<<
                data.second.Ot.first[0], data.second.Ot.second.first[0], data.second.Ot.second.second[0],
                data.second.Ot.first[1], data.second.Ot.second.first[1], data.second.Ot.second.second[1],
                data.second.Ot.first[2], data.second.Ot.second.first[2], data.second.Ot.second.second[2];
            //Eigen::Matrix4f{data.second.Or * model_lrf.Or};
            Eigen::Matrix3f R_1{scene_lrf_Or.transpose()*model_lrf_Or};
            std::cout<<R_1<<std::endl;
        }else{
          continue;
        }

      }
    }
  }
}
