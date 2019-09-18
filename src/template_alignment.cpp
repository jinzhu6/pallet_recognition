#include <template_alignment.h>

// Align a collection of object templates to a sample point cloud

bool Segment_pallet::template_alignment(){
// Load the object templates specified in the object_templates.txt file
  std::vector<FeatureCloud> object_templates;
    std::cout<<"**********"<<std::endl;
  std::ifstream input_stream(model_file_path);
  object_templates.resize(0);
  std::string pcd_filename;
  while (input_stream.good())
  {
    std::getline(input_stream, pcd_filename);
    if (pcd_filename.empty() || pcd_filename.at(0) == '#') // Skip blank lines or comments
      continue;

    FeatureCloud template_cloud;
    template_cloud.loadInputCloud(pcd_filename);
    std::cout<<"pcd_filename: " <<pcd_filename<<std::endl;
    object_templates.push_back(template_cloud);
  }
  input_stream.close();

  // Load the target cloud PCD file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*pallet, *cloud);

  // Assign to the target FeatureCloud
  FeatureCloud target_cloud;
  target_cloud.setTargetInputCloud(cloud);

  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  for (size_t i = 0; i < object_templates.size(); ++i)
  {
    template_align.addTemplateCloud(object_templates[i]);
  }
  template_align.setTargetCloud(target_cloud);

  // Find the best template alignment
  TemplateAlignment::Result best_alignment;
  int best_index = template_align.findBestAlignment(best_alignment);
//  std::cout << best_index << std::endl;
  const FeatureCloud &best_template = object_templates[best_index];
  //copy the best-matched point cloud into model
  model = boost::make_shared<PointCloud_>();
  pcl::copyPointCloud(*best_template.getPointCloud(), *model);
  std::cout << "model has " << model -> size() << "points" << std::endl;
  std::cout << "scene has " << pallet -> size() << "points" << std::endl;

  sac_trans = best_alignment.final_transformation;
  std::cout<<sac_trans<<endl;
  printf("Best fitness score: %f\n", best_alignment.fitness_score);

    pcl::visualization::PCLVisualizer viewer_scene ("Viewer_template");
    pcl::visualization::PointCloudColorHandlerCustom<PointType> viewer_scenel_color_handler (model, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> viewer_ss_color_handler (pallet, 0, 0, 255);
    viewer_scene.addPointCloud (model, viewer_scenel_color_handler, "Viewer_template");
    viewer_scene.addPointCloud (pallet, viewer_ss_color_handler, "Viewer_template2");
    while (!viewer_scene.wasStopped ())
    {
        viewer_scene.spinOnce ();
    }


    // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_writer(new pcl::PointCloud<pcl::PointXYZ>);
    //
    // pcl::copyPointCloud(*model, *pcl_writer);
    //
    // pcl::io::savePCDFile("/home/yuechen/intern/assemble/data/model/model_cloud.pcd", *pcl_writer, true);
    // std::cout<<"scene size: : "<<pcl_writer->size()<<std::endl;
    return true;
}
