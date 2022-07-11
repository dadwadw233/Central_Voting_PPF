#include "CentralVoting.h"
#include "pcl/io/pcd_io.h"

int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr model(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PCDReader reader;

  reader.read(argv[1], *model);

  CentralVoting handle(model);
  handle.CenterExtractor();
  return 0;
}
