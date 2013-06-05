#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

using namespace pcl;

int find_min(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr min_it, int xflag, int yflag, int zflag)
{
  float min;
  //PointCloud<PointXYZRGB>::Ptr min_it(new PointCloud<PointXYZRGB>);
  
  if (xflag)
	min = cloud->points.begin()->x;
  else if (yflag)
	min = cloud->points.begin()->y;
  else if (zflag)
	min = cloud->points.begin()->z;
  
  for (vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> >::const_iterator it = cloud->points.begin(); it != cloud->points.end(); ++it) {
	//cout << "cluster " << j << ": x = " << it1->x << " y = " << it1->y << " z = " << it1->z << endl;
	if (xflag && it->x < min) {
	  min = it->x;
	  min_it = it;
	} else if (yflag && it->y < min) {
	  min = it->y;
	  min_it = it;
	} else if (zflag && it->z < min) {
	  min = it->z;
	  min_it = it;
	}
  }

  return (0);
}