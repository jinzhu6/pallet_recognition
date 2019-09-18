#include <planar_seg.h>


bool Segment_pallet::planar_seg()
{
  if (top_height > 0.4) return (0);
  std::cout << "-------------------------remove the ground-------------------------------" << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pallet_after(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*pallet, *pallet_after);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); //存储输出模型系数
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices); //存储内点，使用的点
	pcl::SACSegmentation<pcl::PointXYZ> seg;

	seg.setOptimizeCoefficients (true);      //设置对估计的模型参数进行优化处理
	seg.setModelType (pcl::SACMODEL_PLANE); //设置模型类型，检测平面
	seg.setMethodType (pcl::SAC_RANSAC);      //设置方法【聚类或随机样本一致性】
	seg.setDistanceThreshold (0.07);    //阈值不要设置太大
	seg.setInputCloud (pallet_after);
	seg.segment (*inliers, *coefficients);    //分割操作

	// Extract the planar inliers from the input cloud
	pcl::ExtractIndices<pcl::PointXYZ>  extract;
	extract.setInputCloud(pallet_after);
	extract.setIndices(inliers);
	//除去平面之外的数据
	extract.setNegative(true);
	extract.filter(*pallet_after);
	pcl::copyPointCloud(*pallet_after, *pallet);

  return (0);
}
