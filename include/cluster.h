/*
 *  cluster.h
 *  
 *
 *  Created by Megha Gupta on 12/20/11.
 *  Copyright 2011 USC. All rights reserved.
 *
 */

#ifndef CLUSTER_H
#define CLUSTER_H

#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <tf/transform_listener.h>

#include "duplo_v0/extract_color_clusters.h"
//#include <pcl/visualization/cloud_viewer.h>

int cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr o_cloud);

#endif