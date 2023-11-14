#include <pcl/console/parse.h>
#include "CentralVoting.h"
#include "add_gauss_noise.h"
#include "pcl/io/pcd_io.h"
int main(int argc, char** argv) {
  if (argc <= 1) {
    PCL_ERROR("Syntax: ./central_voting pcd_model_list pcd_scene(optional)\n");
    return -1;
  }
  std::vector<int> pcd_file_indices =
      pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
  if (pcd_file_indices.size() < 1) {
    PCL_ERROR("need pcd file as input\n");
    return -1;
  }
  float a,b;
  cin>>a>>b;
  pcl::PointCloud<pcl::PointXYZ>::Ptr model(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PCDReader reader;
  reader.read(argv[1], *model);
  reader.read(argv[2], *scene);
  std::cout << argv[1] << " " << argv[2] << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mix(new pcl::PointCloud<pcl::PointXYZ>());
  Eigen::Matrix4f T;
  T << 1, 0, 0, -94,
      0, 1, 0, -428,
      0, 0, 1, -140,
      0, 0, 0, 1;
  Eigen::Affine3f T_(T);
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*model, *model_, T_);
  *mix = *scene + *model_;
  AddGaussNoise agn;  //创建高斯噪声对象agn
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZ>());
  ;                           //保存结果的点云
  agn.setInputCloud(*scene);  //设置输入点云
  agn.setParameters(0, 1);    //设置高斯噪声参数mu,sigma
  agn.addGaussNoise(*cloud_out);
  std::cout << "scene size: " << cloud_out->points.size() << std::endl;
  CentralVoting handle(scene, model);
  handle.CenterExtractorAll();
  handle.setNormalEstimationRadius(4.0f);

  handle.setDownSampleStep(a);
  handle.setAngleThreshold(b);
  handle.setSimpleDownSampleLeaf(Eigen::Vector4f(4.0f, 4.0f, 4.0f, 0.0f));
  handle.setAdaptiveDownSampleOption(false, 20000, 4.0f);
  //handle.test();
  handle.Solve();
  return 0;
}
