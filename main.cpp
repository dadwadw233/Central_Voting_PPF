#include <pcl/console/parse.h>
#include "CentralVoting.h"
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr model(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PCDReader reader;

  reader.read(argv[1], *model);

  CentralVoting handle(model);
  // handle.CenterExtractorAll();
  handle.setNormalEstimationRadius(10.0f);
  handle.setDownSampleStep(10.0f);
  handle.setAngleThreshold(30);
  handle.Solve();
  return 0;
}
