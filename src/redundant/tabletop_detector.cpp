
tabletop_object_detector::TabletopDetection detection_call;
  //we want recognized database objects returned
  //set this to false if you are using the pipeline without the database
  detection_call.request.return_clusters = true;
  //we want the individual object point clouds returned as well
  detection_call.request.return_models = true;
  if (!object_detection_srv.call(detection_call))
  {
    ROS_ERROR("Tabletop detection service failed");
    return -1;
  }
  if (detection_call.response.detection.result != 
      detection_call.response.detection.SUCCESS)
  {
    ROS_ERROR("Tabletop detection returned error code %d", 
              detection_call.response.detection.result);
    return -1;
  }
  if (detection_call.response.detection.clusters.empty() && 
      detection_call.response.detection.models.empty() )
  {
    ROS_ERROR("The tabletop detector detected the table, but found no objects");
    return -1;
  }