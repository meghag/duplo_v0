#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "duplo_ros/Process_PCD.h"
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>

#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
//#include <vector>

//#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;

int planar_seg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr orig_cloud, 
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud, 
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr o_cloud,
               char* fname1, char* fname2);

int read_pcd(char* filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd);

int pass_through(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_filtered);

int cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr o_cloud);

bool process_ros(duplo_ros::Process_PCD::Request &req,
                 duplo_ros::Process_PCD::Response &res)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudz(new pcl::PointCloud<pcl::PointXYZRGB>);	//Filtered cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr planar_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster1(new pcl::PointCloud<pcl::PointXYZRGB>);

  /* ******************* Reading PCD file ***************************/
  char* pcd_name = new char[req.fname.size()+1];
  strcpy(pcd_name, req.fname.c_str());
  if (read_pcd(pcd_name, cloud) == -1)
  {
    std::cerr << "Could not read file " << req.fname << std::endl;
	res.success = 0;
    return false;
  }  

  /****************** Filter out the non-table points ******************/
  if (pass_through(cloud,cloudz) != 0)
    {
      std::cerr << "Pass-through filter error" << std::endl;
	  res.success = 0;
      return false;
    }
	
/* ********* Segmentation of the cloud into table and object clouds **********/
  planar_seg(cloudz,planar_cloud,object_cloud,"table_try.pcd","object_try.pcd");
	 
/* ******************* Visualize Planar Segmentation ***************************/

   /*pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   viewer.showCloud (object_cloud);
   while (!viewer.wasStopped ())
   {
   }
*/
  /********************** Cluster objects based on Euclidean distance **********/
  //vector<double> volumes = cluster(object_cloud);
  cluster(object_cloud);
  //for (vector<double>::iterator it = volumes.begin(); it != volumes.end(); ++it)
//	cerr << *it << endl;

  /************** Parse the clusters **************/
   //Cluster 1
  if (read_pcd("cluster_1.pcd", cluster1) == 1)
  {
    std::cerr << "Couldn't read file cluster_1.pcd" << std::endl;
	res.success = 0;
    return false;
  }
  /*viewer.showCloud (cluster1);
  while (!viewer.wasStopped ())
  {
  }*/

  res.success = 1;
  return true;
}  

int
  main (int argc, char** argv)
{
  ros::init(argc, argv, "process_pcd_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("process_pcd", process_ros);
  ROS_INFO("Ready to process input point cloud data.");
  ros::spin();

  return (0);
}
