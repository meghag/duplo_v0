#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

//#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;

//int find_length(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr min_it, int xflag, int yflag, int zflag);

/*
bool my_x_min (PointXYZRGB i, PointXYZRGB j) { return (i.x < j.x); }
bool my_y_min (PointXYZRGB i, PointXYZRGB j) { return (i.y < j.y); }
bool my_x_max (PointXYZRGB i, PointXYZRGB j) { return (i.x > j.x); }
bool my_y_max (PointXYZRGB i, PointXYZRGB j) { return (i.y > j.y); }
*/

void extract_color_clusters (const PointCloud<PointXYZRGB> &cloud, 
                               const std::vector<int> &indices,
                                const boost::shared_ptr<KdTree<PointXYZRGB> > &tree,
                                float tolerance, std::vector<PointIndices> &clusters,
                                unsigned int min_pts_per_cluster, 
                                unsigned int max_pts_per_cluster);


//vector<double> cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr o_cloud)
int cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr o_cloud)
{
  PCDWriter writer;
  
  // Creating the KdTree object for the search method of the extraction
  KdTree<PointXYZRGB>::Ptr tree (new KdTreeFLANN<PointXYZRGB>);
  tree->setInputCloud (o_cloud);

  cerr << "OK 1 " << endl;

  vector<PointIndices> cluster_indices;

  initTree(0, tree);
  
  vector<int> temp;
  for (size_t j = 0; j < (int)o_cloud->points.size(); ++j)
	temp.push_back(j);
  boost::shared_ptr<const vector<int> > indices (new vector<int> (temp)); 
  
  tree->setInputCloud (o_cloud, indices);
  cerr << "OK 2" << endl;

  /*for (size_t j = 0; j < (*tree->getIndices()).size(); ++j)
	cerr << (*tree->getIndices())[j] << " ";
  cerr<<endl;
  */
  
  /*
  EuclideanClusterExtraction<PointXYZRGB> ec;
  ec.setClusterTolerance (0.01); // 1cm
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (1000);
  ec.setSearchMethod (tree);
  ec.setInputCloud(o_cloud);
  ec.extract (cluster_indices);
*/
  
  extract_color_clusters(*o_cloud, *indices, tree, 0.01, cluster_indices, 30, 2000);

  string fields = getFieldsList(*o_cloud);
  cerr << fields << ", " << fields.size() << endl;
  cerr << "OK 3" << endl;
  
  vector<double> volumes;
  int j = 0;
  PointCloud<PointXYZRGB> combined_cloud;
  
  for (vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    PointCloud<PointXYZRGB>::Ptr cloud_cluster (new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cluster_projected (new PointCloud<PointXYZRGB>);
	

	/*************** Separating out and saving each cluster ***************/
    for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	  cloud_cluster->points.push_back (o_cloud->points[*pit]); //*
	cerr << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << endl;
    stringstream ss;
    ss << "cluster_" << j << ".pcd";
    writer.write<PointXYZRGB> (ss.str (), *cloud_cluster, false); //*

	/***************** Projecting on the X-Y plane ******************/
	ModelCoefficients::Ptr coefficients (new ModelCoefficients ());
	coefficients->values.resize (4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;

	ProjectInliers<PointXYZRGB> proj;
	proj.setModelType (SACMODEL_PLANE);
	proj.setInputCloud (cloud_cluster);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cluster_projected);

	if (j == 0)
	  combined_cloud = *cluster_projected;
	else
	  combined_cloud += *cluster_projected;
	

	stringstream ss2;
	ss2 << "proj_" << j << ".pcd";
    writer.write<PointXYZRGB> (ss2.str (), *cluster_projected, false); //*
	
	j++;
   }
   pcl::io::savePCDFileASCII ("combined_cloud.pcd", combined_cloud);
  
//  return volumes;
  return (j);
}