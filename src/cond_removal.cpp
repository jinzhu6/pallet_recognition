#include <segment_common.h>
#include <cond_removal.h>

bool Segment_pallet::cond_removal() {
  //read parameters from txt file
  std::vector<float> parameters;
  ifstream in("/home/yuechen/intern/assemble/method2/config/cond_removal_parameters.txt");
  if (!in.is_open()) {
    std::cout << "Error opening parameters.txt";
    exit(1);
  }
  float d;
  std::string s;
  while (in >> s >> d) {
    parameters.push_back(d);
  }
  in.close();

  //bottom_height is declared in segment_common.h
  float bottom_height = parameters[0];
  top_height = parameters[1];

  // build the condition
  pcl::ConditionAnd<PointType>::Ptr range_cond (new pcl::ConditionAnd<PointType> ());
  range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
    pcl::FieldComparison<PointType> ("y", pcl::ComparisonOps::GT, bottom_height)));
  range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
    pcl::FieldComparison<PointType> ("y", pcl::ComparisonOps::LT, top_height)));
  // build the filter
  pcl::ConditionalRemoval<PointType> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (scene);
  condrem.setKeepOrganized(true);
  // apply filter
  condrem.filter (*pallet);
  pcl::visualization::PCLVisualizer viewer_seg ("Segment points");
  pcl::visualization::PointCloudColorHandlerCustom<PointType> seg_scence_model_color_handler (pallet, 255, 255, 255);
  viewer_seg.addPointCloud (pallet, seg_scence_model_color_handler, "Segment points");
  while (!viewer_seg.wasStopped ())
  {
      viewer_seg.spinOnce ();
  }
  return (0);
}
