#include <iostream>
#include <string>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

using namespace std;
using namespace pcl;

int read_pcd(char* filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd)
{ 
  /* ******************* Reading PCD file ***************************/
  sensor_msgs::PointCloud2 cloud_blob;
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  if (pcl::io::loadPCDFile (filename, cloud_blob) == -1)
  {
    std::cerr << "Couldn't read file " << filename << std::endl;
    return (-1);
  }
  cerr << "Loaded " << cloud_blob.width * cloud_blob.height
            << " data points from test_pcd.pcd with the following fields: "
            << pcl::getFieldsList (cloud_blob) << endl;

  // Convert to the templated message type
  pcl::fromROSMsg (cloud_blob, *pcd);

  std::cerr << "Point cloud size = "<< pcd->points.size() << std::endl;
  for (size_t i = 0; i < 10; ++i)
	std::cerr << "    " << pcd->points[i].x
              << " " << pcd->points[i].y
              << " " << pcd->points[i].z << std::endl;
 
  return (0);
}