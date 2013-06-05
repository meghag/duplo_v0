/*
 *  extract_color_clusters.h
 *  
 *
 *  Created by Megha Gupta on 12/20/11.
 *  Copyright 2011 USC. All rights reserved.
 *
 */

#ifndef EXTRACT_COLOR_CLUSTERS_H
#define EXTRACT_COLOR_CLUSTERS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

using namespace pcl;

void extract_color_clusters (const pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
							 const std::vector<int> &indices,
							 const boost::shared_ptr<KdTree<pcl::PointXYZRGB> > &tree,
							 float tolerance, std::vector<PointIndices> &clusters,
							 unsigned int min_pts_per_cluster, 
							 unsigned int max_pts_per_cluster);
#endif