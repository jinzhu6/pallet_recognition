#include <segment_common.h>


int main(int argc, char** argv)
{
    Segment_pallet *sp = new Segment_pallet();


    std::string model_file_path = std::string(argv[1]);
    std::string scene_file = std::string(argv[2]);
    std::cout<<model_file_path<<std::endl;
    std::cout<<scene_file<<std::endl;

    sp->initialize(model_file_path, scene_file);

//  remove point cloud in the scene based on height
    sp->cond_removal();

//  sac-icp
    sp->template_alignment();
    sp->icp_pallet();

//  remove ground
//    if (bottom_height <= 0.1) 
    sp->planar_seg();



//  remove scene based on width and length to get pallet
    sp->cond_removal_after();

    std::cout << "-------------------------extract border-------------------------------" << std::endl;

//   extract the border
    sp->extract_border();

    return 0;
}
