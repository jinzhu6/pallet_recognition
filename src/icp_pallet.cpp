#include <icp_pallet.h>
#include <pcl/common/transforms.h>
#include <pcl/console/time.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/common/common.h>
bool Segment_pallet::icp_pallet(){
  //read parameters from txt file
    std::vector<float> parameters;
    ifstream in("/home/yuechen/intern/assemble/method2/config/icp_parameters.txt");
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
    pcl::visualization::PCLVisualizer viewer_icp ("icp Visualization");
//    viewer_icp.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);
    float maxCorrespondenceDistance = parameters[0];
    int maxIterations = (int) parameters[1];
    float transformationEpsilon = parameters[2];
    float euclideanFitnessEpsilon = parameters[3];

    cout << "--- ICP ---------" << endl;
    PointCloud_::Ptr icp_result;
    icp_result = boost::make_shared<PointCloud_>();
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputSource(model);
    icp.setInputTarget(pallet);
    //Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (maxCorrespondenceDistance);
    // 最大迭代次数
    icp.setMaximumIterations (maxIterations);
    // 两次变化矩阵之间的差值
    icp.setTransformationEpsilon (transformationEpsilon);
    // 均方误差
    icp.setEuclideanFitnessEpsilon (euclideanFitnessEpsilon);
    icp.align(*icp_result,sac_trans);
    std::cout << "ICP has converged:" << icp.hasConverged()
        << " score: " << icp.getFitnessScore() << std::endl;
    Eigen::Matrix4f icp_trans;
    icp_trans=icp.getFinalTransformation();
    pcl::transformPointCloud(*model, *icp_result, icp_trans);
    cout << "-----------------" << endl << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*icp_result, *cloud);

    pcl::getMinMax3D(*cloud,min,max);
//    std::cout << min.x << " " << max.x << " " << min.y << " " << max.y << " " << min.z << " " << max.z << std::endl;

    pcl::visualization::PCLVisualizer viewer("registration Viewer");
    //viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
   // viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
    //pcl::visualization::PointCloudColorHandlerCustom<PointType> src_h (model, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> tgt_h (pallet, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> final_h (icp_result, 0, 0, 255);
 //   viewer.addPointCloud (pcd_src, src_h, "source cloud");
    viewer.addPointCloud (pallet, tgt_h, "tgt cloud");
    viewer.addPointCloud (icp_result, final_h, "final cloud");
    //viewer.addCoordinateSystem(1.0);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return true;
}
