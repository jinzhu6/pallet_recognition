#include <segment_common.h>
#include <cond_removal_after.h>

bool Segment_pallet::cond_removal_after() {
  std::cout << "-------------------------remove based on length and width-------------------------------" << std::endl;

  // build the condition
  pcl::ConditionAnd<PointType>::Ptr range_cond (new pcl::ConditionAnd<PointType> ());
  range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
    pcl::FieldComparison<PointType> ("x", pcl::ComparisonOps::GT, min.x)));
  range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
    pcl::FieldComparison<PointType> ("x", pcl::ComparisonOps::LT, max.x)));
  // build the filter
  pcl::ConditionalRemoval<PointType> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (pallet);
  condrem.setKeepOrganized(true);

  // apply filter
  condrem.filter (*pallet);

  pcl::ConditionAnd<PointType>::Ptr range_cond_1 (new pcl::ConditionAnd<PointType> ());
  range_cond_1->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
    pcl::FieldComparison<PointType> ("z", pcl::ComparisonOps::GT, min.z)));
  range_cond_1->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
    pcl::FieldComparison<PointType> ("z", pcl::ComparisonOps::LT, max.z)));
  // build the filter
  pcl::ConditionalRemoval<PointType> condrem_1;
  condrem_1.setCondition (range_cond_1);
  condrem_1.setInputCloud (pallet);
  condrem_1.setKeepOrganized(true);

  // apply filter
  condrem_1.filter (*pallet);
  pcl::visualization::PCLVisualizer viewer_seg ("Segment points");
  pcl::visualization::PointCloudColorHandlerCustom<PointType> seg_scene_model_color_handler (pallet, 255, 255, 255);
  viewer_seg.addPointCloud (pallet, seg_scene_model_color_handler, "Segment points");
  while (!viewer_seg.wasStopped ())
  {
      viewer_seg.spinOnce ();
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_writer(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::copyPointCloud(*pallet, *pcl_writer);

  pcl::io::savePCDFile("/home/yuechen/intern/extract_border/build/cloud.pcd", *pcl_writer, true);

}
