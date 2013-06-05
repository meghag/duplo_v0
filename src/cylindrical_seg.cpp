#include "cylindrical_seg.h"

using namespace std;
using namespace pcl;

int cylindrical_seg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr orig_cloud, 
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr c_cloud, 
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr o_cloud,
                    char* fname1, char* fname2)
{	
	/******************** Cylindrical Segmentation ***************************/

	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	
	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_CYLINDER);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight (0.1);
	seg.setMaxIterations (10000);
	seg.setDistanceThreshold (0.05);
	seg.setRadiusLimits (0, 0.1);
	seg.setInputCloud (orig_cloud);
	seg.setInputNormals (cloud_normals2);

	// Obtain the cylinder inliers and coefficients
	seg.segment (*inliers_cylinder, *coefficients_cylinder);
	std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

	// Write the cylinder inliers to disk
	extract.setInputCloud (cloud_filtered2);
	extract.setIndices (inliers_cylinder);
	extract.setNegative (false);
	pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
	extract.filter (*cloud_cylinder);
	if (cloud_cylinder->points.empty ()) 
		std::cerr << "Can't find the cylindrical component." << std::endl;
	else
	{
		std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
		writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
	}
	return (0);
}