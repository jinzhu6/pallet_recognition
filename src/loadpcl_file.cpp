#include <segment_common.h>
//#include <boost/make_shared.hpp>


Segment_pallet::Segment_pallet(){}

bool Segment_pallet::initialize(std::string model_filename_, std::string scene_filename_){


    model_file_path = model_filename_;
    //        model = boost::make_shared<PointCloud_>();
    scene = boost::make_shared<PointCloud_>();
    pallet = boost::make_shared<PointCloud_>();
    if (pcl::io::loadPCDFile (scene_filename_, *scene) < 0)
    {
        std::cout << "Error loading scene cloud." << std::endl;
        return false;
    }

    //remove NaN points
    std::vector<int> indices;
    //    pcl::removeNaNFromPointCloud(*model,*model, indices);
    //    indices.clear();
    pcl::removeNaNFromPointCloud(*scene,*scene, indices);

    std::cout << "PointCloud before filtering has: " << scene->points.size ()  << " data points." << std::endl;

/*#if 1

    pcl::visualization::PCLVisualizer viewer_scene ("Viewer_scene");
    pcl::visualization::PointCloudColorHandlerCustom<PointType> viewer_scenel_color_handler (scene, 255, 255, 255);
    viewer_scene.addPointCloud (scene, viewer_scenel_color_handler, "Viewer_scene");
    while (!viewer_scene.wasStopped ())
    {
        viewer_scene.spinOnce ();
    }
#endif */
    return true;
}
