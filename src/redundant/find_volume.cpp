#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;

int find_volume(char* filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd)
{ 
  vector<double> vecx, vecy, vecz;
  for (vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> >::const_iterator it1 = cloud_cluster->points.begin(); it1 != cloud_cluster->points.end(); ++it1) {
	//cout << "cluster " << j << ": x = " << it1->x << " y = " << it1->y << " z = " << it1->z << endl;
	vecx.push_back(it1->x);
	vecy.push_back(it1->y);
	vecz.push_back(it1->z);
  }
  *min_element(vecx.begin(), vecx.end());
  
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