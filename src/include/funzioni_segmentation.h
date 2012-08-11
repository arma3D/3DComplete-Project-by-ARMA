#include <pcl/io/pcd_io.h>
#include <limits>
#include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/keypoints/harris_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/shot_omp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/gp3.h>
#include <pcl/keypoints/sift_keypoint.h>

void segmentationAntenna(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source, pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented, float  distanceThreshold,bool output)
{
	// fit plane and keep points above that plane
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (distanceThreshold);

	seg.setInputCloud (source);
	seg.segment (*inliers, *coefficients);

	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud (source);
	extract.setIndices (inliers);
	extract.setNegative (true);

	extract.filter (*segmented);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*segmented, *segmented, indices);

	cout << "axis" << seg.getAxis() << endl;
	size_t n = source->size ();
	//pcl::console::print_info ("Subtracted %zu points along the detected plane\n", n - segmented->size ());
	// Visualize the result
	//pcl::console::print_info ("Segmentend Point cloud... Close window to exit.\n");
	if (output)
	{
		pcl::visualization::PCLVisualizer vis;

		vis.addPointCloud (segmented);
		vis.resetCamera ();
		vis.spin ();
	}
}

void getPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source, pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented, float  distanceThreshold, bool output, pcl::ModelCoefficients::Ptr coefficients)
{
	// fit plane and keep points above that plane
	//pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
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

	cout << "axis" << seg.getAxis() << endl;
	size_t n = source->size ();
	//pcl::console::print_info ("Subtracted %zu points along the detected plane\n", n - segmented->size ());
	// Visualize the result
	//pcl::console::print_info ("Segmentend Point cloud... Close window to exit.\n");
	if (output)
	{
		pcl::visualization::PCLVisualizer vis;

		vis.addPointCloud (segmented);
		vis.resetCamera ();
		vis.spin ();
	}
}

void getAnt(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source, pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented, float  distanceThreshold, bool output, Eigen::Vector3f asse, float eps_angle, pcl::ModelCoefficients::Ptr coefficients)
{
	// fit plane and keep points above that plane
	//pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	//seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(distanceThreshold);

	seg.setInputCloud(source);

	/*seg.setAxis(asse);
	seg.setEpsAngle(eps_angle);*/

	seg.segment(*inliers, *coefficients);

	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(source);
	extract.setIndices(inliers);
	extract.setNegative(false);

	extract.filter(*segmented);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*segmented, *segmented, indices);

	size_t n = source->size ();
	//pcl::console::print_info ("Subtracted %zu points along the detected plane\n", n - segmented->size ());
	// Visualize the result
	//pcl::console::print_info ("Segmentend Point cloud... Close window to exit.\n");
	if (output)
	{
		pcl::visualization::PCLVisualizer vis;

		vis.addPointCloud(segmented);
		vis.resetCamera();
		vis.spin();
	}
}