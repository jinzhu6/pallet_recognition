#include <extract_border.h>


bool Segment_pallet::extract_border()
{
   float angular_resolution = 0.5f;
   pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
   bool setUnseenToMaxRange = true;
   angular_resolution = pcl::deg2rad (angular_resolution);



  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>& point_cloud = *point_cloud_ptr;
  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());  //传感器的位置
  pcl::copyPointCloud(*pallet, *point_cloud_ptr);


  scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                             point_cloud.sensor_origin_[1],
                                                             point_cloud.sensor_origin_[2])) *
                      Eigen::Affine3f (point_cloud.sensor_orientation_);  //仿射变换矩阵


  // -----------------------------------------------
  // -----Create RangeImage from the PointCloud-----
  // -----------------------------------------------


  float noise_level = 0.0;      //各种参数的设置
  float min_range = 0.0f;
  int border_size = 1;
  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians

  boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;
  std::cout << "2.11" << std::endl;

  range_image.createFromPointCloud (point_cloud, angular_resolution, maxAngleWidth, maxAngleHeight,
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  std::cout << "2.2" << std::endl;

  range_image.integrateFarRanges (far_ranges);


  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();

  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");   //创建视口
  viewer.setBackgroundColor (1, 1, 1);                      //设置背景颜色
  viewer.addCoordinateSystem (1.0f);              //设置坐标系
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_cloud_color_handler (point_cloud_ptr, 0, 0, 0);
  viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");   //添加点云
  // pcl::visualization::PointCloudColorHandlerCustom<PointType> viewer_scenel_color_handler (scene, 205, 198, 115);
  // viewer.addPointCloud (scene, viewer_scenel_color_handler, "Viewer_scene");


  // -------------------------
  // -----Extract borders提取边界的部分-----
  // -------------------------
  pcl::RangeImageBorderExtractor border_extractor (&range_image);
  pcl::PointCloud<pcl::BorderDescription> border_descriptions;
  border_extractor.compute (border_descriptions);     //提取边界计算描述子

  // -------------------------------------------------------
  // -----Show points in 3D viewer在3D 视口中显示点云-----
  // ----------------------------------------------------
  pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),  //物体边界
                                            veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),     //veil边界
                                            shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);   //阴影边界
  pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
                                      & veil_points = * veil_points_ptr,
                                      & shadow_points = *shadow_points_ptr;

  for (int y=0; y< (int)range_image.height; ++y)
  {
    for (int x=0; x< (int)range_image.width; ++x)
    {
      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
        border_points.points.push_back (range_image.points[y*range_image.width + x]);

      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
        veil_points.points.push_back (range_image.points[y*range_image.width + x]);

      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
        shadow_points.points.push_back (range_image.points[y*range_image.width + x]);
    }
  }
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler (border_points_ptr, 0, 255, 0);
  viewer.addPointCloud<pcl::PointWithRange> (border_points_ptr, border_points_color_handler, "border points");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler (veil_points_ptr, 255, 0, 0);
  viewer.addPointCloud<pcl::PointWithRange> (veil_points_ptr, veil_points_color_handler, "veil points");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler (shadow_points_ptr, 0, 255, 255);
  viewer.addPointCloud<pcl::PointWithRange> (shadow_points_ptr, shadow_points_color_handler, "shadow points");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");


  pcl::visualization::PointCloudColorHandlerCustom<PointType> viewer_handler (scene, 0, 0, 0);
  viewer.addPointCloud (scene, viewer_handler, "viewer_scene");
  //-------------------------------------
  // -----Show points on range image-----
  // ------------------------------------
//  pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;
//  range_image_borders_widget =
//    pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget (range_image, -std::numeric_limits<float>::infinity (), std::numeric_limits<float>::infinity (), false,
//                                                                          border_descriptions, "Range image with borders");
  // -------------------------------------


  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer.wasStopped ())
  {
  //  range_image_borders_widget->spinOnce ();
    viewer.spinOnce ();
    pcl_sleep(0.01);
  }
}
