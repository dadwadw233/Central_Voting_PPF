#include <pcl/console/parse.h>
#include "CentralVoting.h"
#include "pcl/io/pcd_io.h"
#include "add_gauss_noise.h"
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr model(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PCDReader reader;
  reader.read(argv[1], *model);
  reader.read(argv[2], *scene);
  std::cout << argv[1] << " " << argv[2] << std::endl;
  /*AddGaussNoise agn;							//创建高斯噪声对象agn
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZ>());;	//保存结果的点云
  agn.setInputCloud(*scene);				//设置输入点云
  agn.setParameters(0,1);						//设置高斯噪声参数mu,sigma
  agn.addGaussNoise(*cloud_out);
  std::cout<<"scene size: "<<cloud_out->points.size()<<std::endl;*/
  for(auto &i:scene->points){
    i.x*=4;
    i.y*=4;
    i.z*=4;
  }
  CentralVoting handle(scene, model);
  handle.CenterExtractorAll();
  handle.setNormalEstimationRadius(16.0f);
  handle.setDownSampleStep(16.0f);
  handle.setAngleThreshold(20);
  handle.setSimpleDownSampleLeaf(Eigen::Vector4f(14.0f, 14.0f, 14.0f, 0.0f));
  handle.setAdaptiveDownSampleOption(false, 20000,4.0f);
  //handle.test();
  handle.Solve();
  return 0;
}
