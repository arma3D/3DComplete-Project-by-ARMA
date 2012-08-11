#include <pcl/io/pcd_io.h>
#include <limits>
#include <Eigen/Core>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

void get_a_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source, pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented, float  distanceThreshold, bool output, pcl::ModelCoefficients::Ptr coefficients)
{
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	seg.setOptimizeCoefficients (true);

	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(distanceThreshold);

	seg.setInputCloud(source);
	seg.segment(*inliers, *coefficients);

	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(source);
	extract.setIndices(inliers);
	extract.setNegative(false);

	extract.filter(*segmented);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*segmented, *segmented, indices);

	if (output)
	{
		pcl::visualization::PCLVisualizer vis;

		vis.addPointCloud (segmented);
		vis.resetCamera ();
		vis.spin ();
	}
}