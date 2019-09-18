//#include <segment_common.h>
#include <correspondence_grouping.h>
typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> PointCloud_;

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;

std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;
std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;

int main(){

    float model_ss_ (0.001f);
    float scene_ss_ (0.02f);
    float rf_rad_ (0.16f);
    float descr_rad_ (0.08f);
    float cg_size_ (0.02f);
    float cg_thresh_ (-5.0f);

    pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());


    std::string modelpath = "/home/dl/model_cloud.pcd";
    std::string scenepath = "/home/dl/add_cloud.pcd";


    pcl::io::loadPCDFile(modelpath, *model);
    pcl::io::loadPCDFile(scenepath,*scene);
    /**
* Compute Normals
*/
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    norm_est.setKSearch (100);
    norm_est.setInputCloud (model);
    norm_est.compute (*model_normals);

    norm_est.setInputCloud (scene);
    norm_est.compute (*scene_normals);


    /**
*  Downsample Clouds to Extract keypoints
*/
    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud (model);
    uniform_sampling.setRadiusSearch (model_ss_);

    uniform_sampling.filter (*model_keypoints);
    std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

    uniform_sampling.setInputCloud (scene);
    uniform_sampling.setRadiusSearch (scene_ss_);

    uniform_sampling.filter (*scene_keypoints);
    std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;










    /**
*  Compute Descriptor for keypoints
*/
    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
    descr_est.setRadiusSearch (descr_rad_);

    descr_est.setInputCloud (model_keypoints);
    descr_est.setInputNormals (model_normals);
    descr_est.setSearchSurface (model);
    descr_est.compute (*model_descriptors);

    descr_est.setInputCloud (scene_keypoints);
    descr_est.setInputNormals (scene_normals);
    descr_est.setSearchSurface (scene);
    descr_est.compute (*scene_descriptors);

    /**
*  Find Model-Scene Correspondences with KdTree
*/
    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
    pcl::KdTreeFLANN<DescriptorType> match_search;
    match_search.setInputCloud (model_descriptors);
    std::vector<int> model_good_keypoints_indices;
    std::vector<int> scene_good_keypoints_indices;

    for (size_t i = 0; i < scene_descriptors->size (); ++i)
    {
        std::vector<int> neigh_indices (1);
        std::vector<float> neigh_sqr_dists (1);
        if (!std::isfinite (scene_descriptors->at (i).descriptor[0]))  //skipping NaNs
        {
            continue;
        }
        int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
        if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
        {
            pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            model_scene_corrs->push_back (corr);
            model_good_keypoints_indices.push_back (corr.index_query);
            scene_good_keypoints_indices.push_back (corr.index_match);
        }
    }
    pcl::PointCloud<PointType>::Ptr model_good_kp (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene_good_kp (new pcl::PointCloud<PointType> ());
    pcl::copyPointCloud (*model_keypoints, model_good_keypoints_indices, *model_good_kp);
    pcl::copyPointCloud (*scene_keypoints, scene_good_keypoints_indices, *scene_good_kp);

    std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;
    //    float rf_rad_ (0.2f);
    //    float descr_rad_ (0.08f);
    //    float cg_size_ (0.01f);
    //    float cg_thresh_ (-5.0f);

    //    for(float cg_thresh_ =-10.0f; cg_thresh_ < -1; cg_thresh_ +=1){
//    for(float rf_rad_ =0.01f; rf_rad_ < 1; rf_rad_ +=0.01){
         for(float descr_rad_ =0.01f; descr_rad_ < 1; descr_rad_ +=0.01){

        /**
*  Clustering
*/

        std::vector < pcl::Correspondences > clustered_corrs;

        if (use_hough_)
        {
            pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
            pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

            pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
            rf_est.setFindHoles (true);
            rf_est.setRadiusSearch (rf_rad_);

            rf_est.setInputCloud (model_keypoints);
            rf_est.setInputNormals (model_normals);
            rf_est.setSearchSurface (model);
            rf_est.compute (*model_rf);

            rf_est.setInputCloud (scene_keypoints);
            rf_est.setInputNormals (scene_normals);
            rf_est.setSearchSurface (scene);
            rf_est.compute (*scene_rf);

            //  Clustering
            pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
            clusterer.setHoughBinSize (cg_size_);
            clusterer.setHoughThreshold (cg_thresh_);
            clusterer.setUseInterpolation (true);
            clusterer.setUseDistanceWeight (false);

            clusterer.setInputCloud (model_keypoints);
            clusterer.setInputRf (model_rf);
            clusterer.setSceneCloud (scene_keypoints);
            clusterer.setSceneRf (scene_rf);
            clusterer.setModelSceneCorrespondences (model_scene_corrs);
            std::cout<< "start recognize!"<<std::endl;
            clusterer.recognize (rototranslations, clustered_corrs);
        }
        else
        {
            pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
            gc_clusterer.setGCSize (cg_size_);
            gc_clusterer.setGCThreshold (cg_thresh_);

            gc_clusterer.setInputCloud (model_keypoints);
            gc_clusterer.setSceneCloud (scene_keypoints);
            gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

            std::cout<< "start recognize!"<<std::endl;
            gc_clusterer.recognize (rototranslations, clustered_corrs);
        }

        /**
* Stop if no instances
*/
        if (rototranslations.size () <= 0)
        {
            cout << "*** No instances found! ***" << endl;
//            return false;
            continue;
        }
        else
        {
            cout << "rf_rad_: " << rf_rad_<< endl << endl;
            cout << "cg_size_: " << cg_size_<< endl << endl;
            cout << "cg_thresh_: " << cg_thresh_<< endl << endl;
            cout << "Recognized Instances: " << rototranslations.size () << endl << endl;
#if 1
            //  Visualization
            //
            pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
            pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_model_color_handler (scene, 255, 255, 255);

            viewer.addPointCloud (scene, scene_model_color_handler, "scene_cloud");


            for (size_t i = 0; i < rototranslations.size (); ++i)
            {
                pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
                pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

                std::stringstream ss_cloud;
                ss_cloud << "instance" << i;

                pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);

                //        std::cout<<"rotated_model:" << rotated_model->size()<<std::endl;

                viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

            }

            while (!viewer.wasStopped ())
            {
                viewer.spinOnce ();
            }
#endif

            //        std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;
            for (size_t i = 0; i < rototranslations.size (); ++i)
            {

                pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
                pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);
                instances.push_back (rotated_model);
                //            pcl::PointCloud<pcl::PointXYZ>::Ptr savepoint (new pcl::PointCloud<pcl::PointXYZ> ());
                //            pcl::copyPointCloud(*rotated_model, *savepoint);
                //            pcl::io::savePCDFile("/home/dl/rotated_model.pcd", *savepoint);
            }

        }
    }

    return true;
}
