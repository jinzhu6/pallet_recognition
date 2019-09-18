#ifndef SEGMENT_COMMON_H
#define SEGMENT_COMMON_H

#include <string>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>





struct CloudStyle
{
    double r;
    double g;
    double b;
    double size;

    CloudStyle (double r,
                double g,
                double b,
                double size) :
        r (r),
        g (g),
        b (b),
        size (size)
    {
    }
};

class Segment_pallet
{
public:
    Segment_pallet();
    bool initialize(std::string model_filename_,
                    std::string scene_filename_);

    bool cond_removal();

    bool cond_removal_after();

    bool icp_pallet();

    bool template_alignment();

    bool momentOfInertia();

    bool extract_border();

    bool planar_seg();
//    void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
//                           void* nothing);

    typedef pcl::PointXYZRGBA PointType;
    typedef pcl::PointCloud<PointType> PointCloud_;


private:
    std::string model_filename_;
    std::string scene_filename_;

    std::string model_file_path;

    PointCloud_::Ptr model,
    scene,
    pallet;

    pcl::PointXYZ min; // store min xyz
    pcl::PointXYZ max; //store max xyz
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;

    std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;
//    std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;
    Eigen::Matrix4f sac_trans;

    float top_height;


};

#endif // SEGMENT_COMMON_H
