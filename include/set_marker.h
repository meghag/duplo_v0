#ifndef SET_MARKER_H 
#define SET_MARKER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker 
set_marker(std::string frame_id, std::string ns, int id, int shape, 
           geometry_msgs::Point position, float scale, float r, float g, float b, float a);

#endif