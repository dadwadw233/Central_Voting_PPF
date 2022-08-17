//
// Created by yyh on 22-7-26.
//

#include "PPFRegistration.h"

PPFRegistration::PPFRegistration() {
  model_cloud_with_normal.reset(new pcl::PointCloud<pcl::PointNormal>());
  scene_cloud_with_normal.reset(new pcl::PointCloud<pcl::PointNormal>());
  searchMap.reset(new Hash::HashMap());
}
void PPFRegistration::setSceneReferencePointSamplingRate(
    const float &scene_reference_point_sampling_rate) {
  this->scene_reference_point_sampling_rate =
      scene_reference_point_sampling_rate;
}

void PPFRegistration::setPositionClusteringThreshold(
    const float &clustering_position_diff_threshold) {
  this->clustering_position_diff_threshold = clustering_position_diff_threshold;
}

void PPFRegistration::setRotationClusteringThreshold(
    const float &clustering_rotation_diff_threshold) {
  this->clustering_rotation_diff_threshold = clustering_rotation_diff_threshold;
}
void PPFRegistration::setInputTarget(
    const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud) {
  this->scene_cloud_with_normal = cloud;
}
void PPFRegistration::setInputSource(
    const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud) {
  this->model_cloud_with_normal = cloud;
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

Eigen::Affine3f PPFRegistration::getFinalTransformation() {
  return this->finalTransformation;
}
template <class T>
typename pcl::PointCloud<T>::Ptr PPFRegistration::aligen(
    const typename pcl::PointCloud<T>::Ptr &input) {
  typename pcl::PointCloud<T>::Ptr output =
      boost::make_shared<pcl::PointCloud<T>>();
  pcl::transformPointCloud(*input, *output, finalTransformation);
  return output;
}
void PPFRegistration::setModelTripleSet(
    const std::vector<pcl::PointXYZ> &triple_set) {
  for (size_t i = 0; i < 3; ++i) {
    this->triple_set.push_back(triple_set[i]);
  }
}
void PPFRegistration::setDobj(const float &data) { this->d_obj = data; }

void PPFRegistration::vote(const key_ &key, const Eigen::Affine3f &T) {
  if (map_.find(key) != map_.end()) {
    (map_.find(key)->second).value += 1;
    (map_.find(key)->second).T_set.push_back(T);
  } else {
    data_ d(T, 1);
    map_.emplace(key, d);
  }
}
void PPFRegistration::vote(const int &key, const Eigen::Affine3f &T) {
  if(map_center.find(key)!=map_center.end()){
    (map_center.find(key)->second).value += 1;
    (map_center.find(key)->second).T_set.push_back(T);
  }else {
    data_ d(T, 1);
    map_center.emplace(key, d);
  }
}

void PPFRegistration::establishVoxelGrid() {
  pcl::PointNormal max_point, min_point;
  pcl::MomentOfInertiaEstimation<pcl::PointNormal> feature_extractor;
  feature_extractor.setInputCloud(this->scene_cloud_with_normal);
  feature_extractor.compute();
  feature_extractor.getAABB(min_point, max_point);
  this->x_range = std::make_pair(min_point.x, max_point.x);
  this->y_range = std::make_pair(min_point.y, max_point.y);
  this->z_range = std::make_pair(min_point.z, max_point.z);
}

decltype(auto) PPFRegistration::HypoVerification(const Eigen::Affine3f &T) {
  pcl::PointCloud<pcl::PointNormal>::Ptr temp =
      boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
  pcl::transformPointCloud(*this->model_cloud_with_normal, *temp, T);
  pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree(
      new pcl::search::KdTree<pcl::PointNormal>());
  auto cnt = 0;
  double radius = 0.02 * this->d_obj;
  kdtree->setInputCloud(this->scene_cloud_with_normal);
  //#pragma omp parallel for shared(temp, radius, cnt,search_cloud, kdtree)
  //default(none) num_threads(15)
  for (auto i = temp->points.begin(); i != temp->points.end(); i++) {
    std::vector<int> indices;
    std::vector<float> distance;
    //#pragma omp critical
    kdtree->radiusSearch(*i, radius, indices, distance);
    if (!indices.empty()) {
      //#pragma omp critical
      cnt +=0 ;
      continue;
    } else {
      int num = 0;
      for (auto j = 0; j < indices.size(); ++j) {
        if (pcl::getAngle3D(static_cast<const Eigen::Vector3f>(
                                scene_cloud_with_normal->points[indices[j]].normal),
                            static_cast<const Eigen::Vector3f>(i->normal),
                            true) >= 25) {
          num++;
          break;
        } else {
          continue;
        }
      }
      if (num > 0) {
        //#pragma omp critical
        cnt++;
      } else {
        //#pragma omp critical
        cnt +=0 ;
      }
    }
  }

  //#pragma omp barrier
  return cnt;
}
decltype(auto) PPFRegistration::HypoVerification(const Eigen::Matrix4f &T) {
  pcl::PointCloud<pcl::PointNormal>::Ptr temp =
      boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
  pcl::transformPointCloud(*this->model_cloud_with_normal, *temp, T);
  pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree(
      new pcl::search::KdTree<pcl::PointNormal>());
  auto cnt = 0;
  double radius = 0.02 * this->d_obj;
  kdtree->setInputCloud(this->scene_cloud_with_normal);
  std::vector<int>nan;
  pcl::PointCloud<pcl::PointNormal>::Ptr temp_ =
      boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
  temp_->is_dense = false;
  pcl::removeNaNFromPointCloud(*temp, *temp_, nan);
  #pragma omp parallel for shared(temp, radius, cnt, kdtree, temp_) default(none) num_threads(15)
  for (auto i = 0; i < temp_->points.size(); i++) {
    std::vector<int> indices;
    std::vector<float> distance;
    #pragma omp critical
    kdtree->radiusSearch(temp_->points[i], radius, indices, distance);
    if (indices.empty()) {
      #pragma omp critical
      cnt += 0;
      continue;
    } else {
      int num = 0;
      for (auto j = 0; j < indices.size(); ++j) {
        if (pcl::getAngle3D(static_cast<const Eigen::Vector3f>(
                                scene_cloud_with_normal->points[indices[j]].normal),
                            static_cast<const Eigen::Vector3f>(temp_->points[i].normal),
                            true) < 25) {

          num++;
          break;
        } else {
          continue;
        }
      }
      if (num > 0) {
        #pragma omp critical
        cnt++;
      } else {
        #pragma omp critical
        cnt += 0;
      }
    }
  }

  #pragma omp barrier
  return cnt;
}
template <class T>
float calculateDistance(T &pointA, T &pointB){
  return sqrt(pow(pointA[0]-pointB[0],2)+pow(pointA[1]-pointB[1],2)+pow(pointA[2]-pointB[2],2));
}
void PPFRegistration::compute() {
  // pcl::PointCloud<pcl::PointXYZ>::Ptr triple_scene =
  // boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  establishVoxelGrid();
  auto xr =
      std::abs(static_cast<float>(this->x_range.second - this->x_range.first));
  auto yr =
      std::abs(static_cast<float>(this->y_range.second - this->y_range.first));
  auto zr =
      std::abs(static_cast<float>(this->z_range.second - this->z_range.first));

  auto x_num = static_cast<long long int>(
      std::ceil(xr / this->clustering_position_diff_threshold));
  auto y_num = static_cast<long long int>(
      std::ceil(yr / this->clustering_position_diff_threshold));
  auto z_num = static_cast<long long int>(
      std::ceil(zr / this->clustering_position_diff_threshold));
  pcl::PPFSignature feature{};
  std::pair<Hash::HashKey, Hash::HashData> data{};
  Eigen::Vector3f p1{};
  Eigen::Vector3f p2{};
  Eigen::Vector3f n1{};
  Eigen::Vector3f n2{};
  Eigen::Vector3f delta{};
  int r_num = 0;
  auto tp1 = boost::chrono::steady_clock::now();
  pcl::PointCloud<pcl::PointXYZ>::Ptr triple_scene(
      new pcl::PointCloud<pcl::PointXYZ>());
  std::cout<<"finish Registering init"<<std::endl;
  std::cout<<"computing ..."<<std::endl;
  int scale_cnt = 0;
  float scale = 0;
  for (auto i = 0; i < scene_cloud_with_normal->points.size(); ++i) {
#pragma omp parallel for shared(                                              \
    x_num, y_num, z_num, zr, xr, yr, i, triple_scene,                         \
    scene_reference_point_sampling_rate, r_num, scale, scale_cnt) private(p1, p2, n1, n2, delta,       \
                                                 feature, data) default(none) \
    num_threads(15)
    for (auto j = 0; j < scene_cloud_with_normal->points.size() / 10; ++j) {
      if (i == j) {
        continue;
      } else {
        // triple_scene.reset();
        p1 << scene_cloud_with_normal->points[i].x,
            scene_cloud_with_normal->points[i].y,
            scene_cloud_with_normal->points[i].z;
        p2 << scene_cloud_with_normal->points[j].x,
            scene_cloud_with_normal->points[j].y,
            scene_cloud_with_normal->points[j].z;
        n1 << scene_cloud_with_normal->points[i].normal_x,
            scene_cloud_with_normal->points[i].normal_y,
            scene_cloud_with_normal->points[i].normal_z;
        n2 << scene_cloud_with_normal->points[j].normal_x,
            scene_cloud_with_normal->points[j].normal_y,
            scene_cloud_with_normal->points[j].normal_z;

        delta = p2 - p1;
        float f4 = delta.norm();
        if (f4 > d_obj) {
          continue;
        }
        /*
        if(f4<250)
        {
          continue;
        }*/
        // normalize
        delta /= f4;

        float f1 = n1[0] * delta[0] + n1[1] * delta[1] + n1[2] * delta[2];

        float f2 = n1[0] * delta[0] + n2[1] * delta[1] + n2[2] * delta[2];

        float f3 = n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2];

        /*float f1 = n1.x() * delta.x() + n1.y()  * delta.y() + n2.z()  *
        delta.z();

        float f2 = n1.x() * delta.x() + n2.y()  * delta.y() + n2.z()  *
        delta.z();

        float f3 = n1.x() * n2.x() + n1.y()  * n2.y()  + n1.z()  * n2.z() ;
         */
        feature.f1 = f1;
        feature.f2 = f2;
        feature.f3 = f3;
        feature.f4 = f4;
        feature.alpha_m = 0.0f;
        data.second.Or =
            (std::make_pair(n1.cross(delta)/(n1.cross(delta)).norm(),
                            std::make_pair(n1.cross(n1.cross(delta))/(n1.cross(n1.cross(delta))).norm(), n1/n1.norm())));

        data.second.Ot =
            (std::make_pair(n2.cross(delta)/(n2.cross(delta)).norm(),
                            std::make_pair(n2.cross(n2.cross(delta))/(n2.cross(n2.cross(delta))).norm(), n2/n2.norm())));

        data.first.k1 =
            static_cast<int>(std::floor(f1 / angle_discretization_step));
        data.first.k2 =
            static_cast<int>(std::floor(f2 / angle_discretization_step));
        data.first.k3 =
            static_cast<int>(std::floor(f3 / angle_discretization_step));
        data.second.dist =
            static_cast<int>(std::floor(f4 / distance_discretization_step));

        data.second.r = scene_cloud_with_normal->points[i];
        data.second.t = scene_cloud_with_normal->points[j];
        if (searchMap->find(data.first)) {

          auto model_lrf = this->searchMap->getData(data.first);

          Eigen::Matrix3f model_lrf_Or;
          Eigen::Matrix3f model_lrf_Ot;
          Eigen::Matrix3f scene_lrf_Or;
          Eigen::Matrix3f scene_lrf_Ot;

          model_lrf_Or << model_lrf.Or.first[0], model_lrf.Or.second.first[0],
              model_lrf.Or.second.second[0], model_lrf.Or.first[1],
              model_lrf.Or.second.first[1], model_lrf.Or.second.second[1],
              model_lrf.Or.first[2], model_lrf.Or.second.first[2],
              model_lrf.Or.second.second[2];
          model_lrf_Ot << model_lrf.Ot.first[0], model_lrf.Ot.second.first[0],
              model_lrf.Ot.second.second[0], model_lrf.Ot.first[1],
              model_lrf.Ot.second.first[1], model_lrf.Ot.second.second[1],
              model_lrf.Ot.first[2], model_lrf.Ot.second.first[2],
              model_lrf.Ot.second.second[2];
          scene_lrf_Or << data.second.Or.first[0],
              data.second.Or.second.first[0], data.second.Or.second.second[0],
              data.second.Or.first[1], data.second.Or.second.first[1],
              data.second.Or.second.second[1], data.second.Or.first[2],
              data.second.Or.second.first[2], data.second.Or.second.second[2];
          scene_lrf_Ot << data.second.Ot.first[0],
              data.second.Ot.second.first[0], data.second.Ot.second.second[0],
              data.second.Ot.first[1], data.second.Ot.second.first[1],
              data.second.Ot.second.second[1], data.second.Ot.first[2],
              data.second.Ot.second.first[2], data.second.Ot.second.second[2];
          // Eigen::Matrix4f{data.second.Or * model_lrf.Or};

          Eigen::Matrix3f R_1{scene_lrf_Or.transpose() * model_lrf_Or};
          Eigen::Matrix3f R_2{scene_lrf_Ot.transpose() * model_lrf_Ot};

          Eigen::Vector4f t_1{};
          Eigen::Vector4f t_2{};
          Eigen::Vector3f m_1{model_lrf.r.x, model_lrf.r.y, model_lrf.r.z};
          Eigen::Vector3f m_2{model_lrf.t.x, model_lrf.t.y, model_lrf.t.z};

          m_1 = R_1 * m_1;
          m_2 = R_1 * m_2;

          t_1 << data.second.r.x - m_1[0], data.second.r.y - m_1[1],
              data.second.r.z - m_1[2], 1.0f;
          t_2 << data.second.t.x - m_2[0], data.second.t.y - m_2[1],
              data.second.t.z - m_2[2], 1.0f;
          // std::cout<<R_1<<std::endl;

          Eigen::Matrix4f T_1{};
          Eigen::Matrix4f T_2{};

          T_1 << R_1(0, 0), R_1(0, 1), R_1(0, 2), t_1[0], R_1(1, 0), R_1(1, 1),
              R_1(1, 2), t_1[1], R_1(2, 0), R_1(2, 1), R_1(2, 2), t_1[2], 0.0f,
              0.0f, 0.0f, t_1[3];
          T_2 << R_2(0, 0), R_2(0, 1), R_2(0, 2), t_2[0], R_2(1, 0), R_2(1, 1),
              R_2(1, 2), t_2[1], R_2(2, 0), R_2(2, 1), R_2(2, 2), t_2[2], 0.0f,
              0.0f, 0.0f, t_1[3];

          pcl::PointXYZ p;
          Eigen::Affine3f transform_1(T_1);
          Eigen::Affine3f transform_2(T_2);
          Eigen::Vector3f model_center{};
          Eigen::Vector3f hypo_center{};
          Eigen::Vector3f hypo_center_{};
          model_center<<triple_set[0].x, triple_set[0].y, triple_set[0].z;
          pcl::transformPoint(model_center, hypo_center, transform_1);
          pcl::transformPoint(model_center, hypo_center_, transform_2);
          if(::calculateDistance(hypo_center, hypo_center_)>50){
            continue;
          }
          std::vector<int> index_1, index_2;
#pragma omp critical
          scale+=data.second.dist/model_lrf.dist;
#pragma omp critical
          ++scale_cnt;
          for (int i = 0; i < 3; i++) {
            Eigen::Vector3f m{};
            Eigen::Vector3f s{};
            m << triple_set[i].x, triple_set[i].y, triple_set[i].z;
            s << 0.0f, 0.0f, 0.0f;
            pcl::transformPoint(m, s, transform_1);
            /*if(isnan(s[0])|| isnan(s[1])||isnan(s[2])){
              break;
            }*/
            int xCell = static_cast<int>(
                            std::ceil((s[0] - this->x_range.first) /
                                      clustering_position_diff_threshold)) == 0
                            ? 1
                            : static_cast<int>(std::ceil(
                                  (s[0] - this->x_range.first) /
                                  clustering_position_diff_threshold));
            int yCell = static_cast<int>(
                            std::ceil((s[1] - this->y_range.first) /
                                      clustering_position_diff_threshold)) == 0
                            ? 1
                            : static_cast<int>(std::ceil(
                                  (s[1] - this->y_range.first) /
                                  clustering_position_diff_threshold));
            int zCell = static_cast<int>(
                            std::ceil((s[2] - this->z_range.first) /
                                      clustering_position_diff_threshold)) == 0
                            ? 1
                            : static_cast<int>(std::ceil(
                                  (s[2] - this->z_range.first) /
                                  clustering_position_diff_threshold));
            if (i == 0) {
#pragma omp critical
              triple_scene->points.emplace_back(s[0], s[1], s[2]);
            }

            index_1.push_back((xCell - 1) + (yCell - 1) * x_num +
                              (zCell - 1) * x_num * y_num);
            pcl::transformPoint(m, s, transform_2);
            /*if(isnan(s[0])|| isnan(s[1])||isnan(s[2])){
              break;
            }*/
            xCell = static_cast<int>(
                        std::ceil((s[0] - this->x_range.first) /
                                  clustering_position_diff_threshold)) == 0
                        ? 1
                        : static_cast<int>(
                              std::ceil((s[0] - this->x_range.first) /
                                        clustering_position_diff_threshold));
            yCell = static_cast<int>(
                        std::ceil((s[1] - this->y_range.first) /
                                  clustering_position_diff_threshold)) == 0
                        ? 1
                        : static_cast<int>(
                              std::ceil((s[1] - this->y_range.first) /
                                        clustering_position_diff_threshold));
            zCell = static_cast<int>(
                        std::ceil((s[2] - this->z_range.first) /
                                  clustering_position_diff_threshold)) == 0
                        ? 1
                        : static_cast<int>(
                              std::ceil((s[2] - this->z_range.first) /
                                        clustering_position_diff_threshold));
            if (i == 0) {
#pragma omp critical
              triple_scene->points.emplace_back(s[0], s[1], s[2]);
            }

            index_2.push_back((xCell - 1) + (yCell - 1) * x_num +
                              (zCell - 1) * x_num * y_num);
          }

          key_ key_1(index_1[0], index_1[1], index_1[2]);
          key_ key_2(index_2[0], index_2[1], index_2[2]);
          //if(fabs(index_1[0]-index_2[0])>10){
          //  continue;
          //}
#pragma omp critical
          this->vote(key_1, transform_1);
#pragma omp critical
          this->vote(key_2, transform_2);
        } else {
          continue;
        }
      }
    }
  }

#pragma omp barrier


  std::cout<<"hypo scale: "<<scale/scale_cnt<<std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr triple(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp(
      new pcl::PointCloud<pcl::PointXYZ>());
  std::vector<int> indices;
  triple_scene->is_dense = false;
  pcl::removeNaNFromPointCloud(*triple_scene, *temp, indices);
  tree->setInputCloud(temp);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(this->clustering_position_diff_threshold);
  ec.setMinClusterSize(2);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(temp);
  ec.extract(cluster_indices);

  key_ final_key(-1, -1, -1);
  int max_vote = 0;
  if(this->map_.empty()){
    std::cout<<"no ans"<<std::endl;
  }else{
    for (const auto &i : this->map_) {
      auto T_mean = getMeanMatrix(i.second.T_set);
      //auto T_mean = i.second.T_set[0].matrix();
      if(isnan(T_mean(0,0))){
        continue;
      }
      auto cnt = HypoVerification(T_mean);

      Eigen::Affine3f temp_(T_mean);
      //std::cout<<temp.matrix()<<std::endl;
      struct data node(temp_, cnt + i.second.value);
      T_queue.push(node);
      if (i.second.value > max_vote) {
        max_vote = i.second.value;
        final_key = i.first;
      } else {
        continue;
      }
    }
    std::cout << "final vote: " << max_vote << std::endl;
    while(isnan(T_queue.top().T(0,0))){
      T_queue.pop();
    }
    std::cout << "max value: " << T_queue.top().value << std::endl;
    this->finalTransformation = T_queue.top().T;
    std::cout << "transform matrix: " << std::endl
              << this->finalTransformation.matrix();
  }

  /**generate cluster **/

    for (auto i = cluster_indices.begin(); i != cluster_indices.end(); ++i) {
      for (auto j = 0; j < i->indices.size(); j++) {
        triple->points.push_back(temp->points[i->indices[j]]);
      }
    }

  /*visualize*/

  std::cout << "\ntriple size: " << temp->size() << std::endl;
  std::cout<<"Transform size: "<<this->map_.size()<<std::endl;

  pcl::visualization::PCLVisualizer view("subsampled point cloud");
  view.setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(
      triple, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> white(
      scene_cloud_with_normal, 255, 255, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> green(
      model_cloud_with_normal, 0, 255, 0);
  view.addPointCloud(triple, red, "triple");
  view.addPointCloud(model_cloud_with_normal, green, "model");
  view.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "triple");
  view.addPointCloud(scene_cloud_with_normal, white, "scene");

  while (!view.wasStopped()) {
    view.spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  }



}
