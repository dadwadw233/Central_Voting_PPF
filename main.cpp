#include <pcl/console/parse.h>
#include "CentralVoting.h"
#include "add_gauss_noise.h"
#include "pcl/io/pcd_io.h"
#include <cstring>

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
  cout<<"Input downsample step (for 16GB RAM, 20 is recommended):"<<endl;
  //cin>>a;
  cout<<"Input downsample angle threshold (for 16GB RAM 20 is recommended):"<<endl;
  //cin>>b;
  a=0.05;
  b=0.05;
  //cin>>a>>b;
  pcl::PointCloud<pcl::PointXYZ>::Ptr model(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PCDReader reader;
  reader.read(argv[1], *model);
  reader.read(argv[2], *scene);
  std::string output_path = argv[1];
  size_t lastSlashPos = output_path.find_last_of('/');
  if (lastSlashPos != std::string::npos) {
    output_path = output_path.substr(0, lastSlashPos);
  }
  cout<<"结果输出路径： "<<output_path<<endl;
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
  handle.setNormalEstimationRadius(0.05f);

  handle.setDownSampleStep(a);
  handle.setAngleThreshold(b);
  handle.setSimpleDownSampleLeaf(Eigen::Vector4f(0.1f, 0.1f, 0.1f, 0.1f));
  handle.setAdaptiveDownSampleOption(false, 20000, 4.0f);
  //handle.test();
  auto result = handle.Solve();
  std::cout<<"预测出的结果数量： "<<result.size()<<endl;
  auto filename = output_path + "/central_voting_results.txt";
  std::ofstream file(filename);

  if (!file.is_open()) {
    std::cerr << "Unable to open file: " << filename << std::endl;
    return -1;
  }
  for (auto item : result){
    auto mat = item.matrix();
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        file << mat(i, j);
        if (j != 3) file << " ";
      }
      if (i != 3) file << " ";
    }
    file << std::endl;

  }
  file.close();
  std::cout<<"结果输出完成"<<endl;
  return 0;
}
