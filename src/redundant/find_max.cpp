#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr find_max(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int xflag, int yflag, int zflag)
{
  float max;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr min_it(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  if (xflag)
	max = cloud->points.begin()->x;
  else if (yflag)
	max = cloud->points.begin()->y;
  else if (zflag)
	max = cloud->points.begin()->z;
  
  for (vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> >::const_iterator it = cloud->points.begin(); it != cloud->points.end(); ++it) {
	//cout << "cluster " << j << ": x = " << it1->x << " y = " << it1->y << " z = " << it1->z << endl;
	if (xflag && it->x > max) {
	  max = it->x;
	  max_it = it;
	} else if (yflag && it->y > max) {
	  max = it->y;
	  max_it = it;
	} else if (zflag && it->z > max) {
	  max = it->z;
	  max_it = it;
	}
  }

  return max_it;
}