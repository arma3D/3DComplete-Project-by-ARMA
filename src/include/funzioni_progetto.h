/*#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>*/
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
#include <pcl/keypoints/harris_keypoint3D.h>
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

class DetailedCloud
{
public:
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
	typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudColor;
	typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
	typedef pcl::search::KdTree<pcl::PointXYZRGB> SearchColorMethod;

	// Costruttore
	DetailedCloud() :
		searchColorMethod_ (new SearchColorMethod),
		normal_radius_(0.02f),
		feature_radius_(0.02f)
	{}

	DetailedCloud(float normal_radius, float feature_radius) :
		searchColorMethod_ (new SearchColorMethod),
		normal_radius_(normal_radius),
		feature_radius_(feature_radius)
	{}

	~DetailedCloud()
	{
	}

	void setInputCloud(PointCloudColor::Ptr cloudPointer)
	{
		cloudPointer_ = cloudPointer;
		processInputCloud();
	}

	void setInputCloudFormPCD(const std::string &pcd_file)
	{
		cloudPointer_ = PointCloudColor::Ptr (new PointCloudColor);
		pcl::io::loadPCDFile (pcd_file, *cloudPointer_);
		processInputCloud();
	}

	PointCloudColor::Ptr getPointCloud() const
	{
		return(cloudPointer_);
	}

	SurfaceNormals::Ptr getSurfaceNormals() const
	{
		return(normals_);
	}

	LocalFeatures::Ptr getLocalFeatures() const
	{
		return(features_);
	}

	float getNormalRadius()
	{
		return(normal_radius_);
	}

	float getFeatureRadius()
	{
		return(feature_radius_);
	}

protected:
	void processInputCloud()
	{
		computeSurfaceNormals();
		computeLocalFeatures();
	}

	void computeSurfaceNormals()
	{
		normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
		norm_est.setInputCloud(cloudPointer_);
		norm_est.setSearchMethod(searchColorMethod_);
		norm_est.setRadiusSearch(normal_radius_);
		norm_est.compute (*normals_);
	}

	void computeLocalFeatures ()
	{
		features_ = LocalFeatures::Ptr (new LocalFeatures);

		pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
		fpfh_est.setInputCloud (cloudPointer_);
		fpfh_est.setInputNormals (normals_);
		fpfh_est.setSearchMethod (searchColorMethod_);
		fpfh_est.setRadiusSearch (feature_radius_);
		fpfh_est.compute (*features_);
	}

private:
	//variabili private
	PointCloudColor::Ptr cloudPointer_;
	SurfaceNormals::Ptr normals_;
	LocalFeatures::Ptr features_;
	SearchColorMethod::Ptr searchColorMethod_;

	float normal_radius_;
	float feature_radius_;
};

class TemplateAlignment
{
public:
	struct Result
	{
		float fitness_score;
		Eigen::Matrix4f final_transformation;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	TemplateAlignment() :
		min_sample_distance_(0.05f),
		max_correspondence_distance_(0.01f*0.01f),
		nr_iterations_(500)
	{
		sac_ia_.setMinSampleDistance (min_sample_distance_);
		sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
		sac_ia_.setMaximumIterations (nr_iterations_);
	}

	TemplateAlignment(float min_sample_distance, float max_correspondence_distance, int nr_iterations) :
		min_sample_distance_(min_sample_distance),
		max_correspondence_distance_(max_correspondence_distance),
		nr_iterations_(nr_iterations)
	{
		sac_ia_.setMinSampleDistance (min_sample_distance_);
		sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
		sac_ia_.setMaximumIterations (nr_iterations_);
	}

	~TemplateAlignment (){}

	void setTargetCloud(DetailedCloud &target_cloud)
	{
		target_ = target_cloud;
		sac_ia_.setInputTarget (target_cloud.getPointCloud ());
		sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
	}

	// Add the given cloud to the list of template clouds
	void addTemplateCloud(DetailedCloud &template_cloud)
	{
		templates_.push_back (template_cloud);
	}

	// Align the given template cloud to the target specified by setTargetCloud ()
	void align(DetailedCloud &template_cloud, TemplateAlignment::Result &result)
	{
		sac_ia_.setInputCloud (template_cloud.getPointCloud ());
		sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

		pcl::PointCloud<pcl::PointXYZRGB> registration_output;

		std::cout << "### FUNCTION align() ### aligning with normal radius " << template_cloud.getNormalRadius() << " and feature radius " << template_cloud.getFeatureRadius() << endl;
		sac_ia_.align(registration_output);

		result.fitness_score = (float)sac_ia_.getFitnessScore(max_correspondence_distance_);
		result.final_transformation = sac_ia_.getFinalTransformation();
	}

	// Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
	void alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
	{
		results.resize (templates_.size ());
		for (size_t i = 0; i < templates_.size (); ++i)
		{
			align(templates_[i], results[i]);
		}
	}

	// Align all of template clouds to the target cloud to find the one with best alignment score
	int findBestAlignment (TemplateAlignment::Result &result)
	{
		// Align all of the templates to the target cloud
		std::vector<Result, Eigen::aligned_allocator<Result> > results;
		alignAll(results);

		// Find the template with the best (lowest) fitness score
		float lowest_score = std::numeric_limits<float>::infinity ();
		int best_template = 0;
		for (size_t i = 0; i < results.size (); ++i)
		{
		const Result &r = results[i];
		if (r.fitness_score < lowest_score)
		{
			lowest_score = r.fitness_score;
			best_template = (int) i;
		}
	}

		// Output the best alignment
		result = results[best_template];
		return (best_template);
	}

private:
		// A list of template clouds and the target to which they will be aligned
		std::vector<DetailedCloud> templates_;
		DetailedCloud target_;

		// The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
		pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia_;
		float min_sample_distance_;
		float max_correspondence_distance_;
		int nr_iterations_;
};

void visualizeTwo(pcl::visualization::PCLVisualizer* viewer, string name_1, string name_2, pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg_source_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg_target_cloud)
{
	int v1(0);
	(*viewer).createViewPort (0.0, 0.0, 0.5, 1.0, v1);
	(*viewer).setBackgroundColor (0.4, 0.4, 0.4, v1);
	(*viewer).addCoordinateSystem (0.1, v1);
	(*viewer).addText ("seg source voxelled", 10, 10, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(seg_source_cloud);
	(*viewer).addPointCloud<pcl::PointXYZRGB> (seg_source_cloud, rgb1, "input_cloud_1",v1);

	int v2(0);
	(*viewer).createViewPort (0.5, 0.0, 1.0, 1.0, v2);
	(*viewer).setBackgroundColor (0.5, 0.5, 0.5, v2);
	(*viewer).addCoordinateSystem (0.1, v2);
	(*viewer).addText ("seg target voxelled", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(seg_target_cloud);
	(*viewer).addPointCloud<pcl::PointXYZRGB> (seg_target_cloud, rgb2, "input_cloud_2",v2);
	(*viewer).resetCamera();
}

void doVoxel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float voxel_grid_size)
{
	pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
	voxel_grid.setInputCloud(cloud);
	voxel_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
	voxel_grid.filter(*cloud);
}

void segmentation (pcl::PointCloud<pcl::PointXYZRGB>::Ptr source, pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented, float  distanceThreshold,bool output)
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

/**
     * @brief Detects key points in the input point cloud
     * @param cloud the input point cloud
     * @param keypoints the resulting key points. Note that they are not necessarily a subset of the input cloud
     * @param min_scale, nr_octaves, nr_scales parameters for setScales()
     * @param min_contrast parameter for setMinimumContrast()
*/
void detect_keypoints(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints,float min_scale, int nr_octaves, int nr_scales, float min_contrast)
{
	pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI> sift_detect;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeK (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	sift_detect.setSearchMethod (treeK);
	sift_detect.setScales (min_scale, nr_octaves, nr_scales);
	sift_detect.setMinimumContrast (min_contrast);
	sift_detect.setInputCloud (cloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_temp(new pcl::PointCloud<pcl:: PointXYZI>);
	sift_detect.compute (*keypoints_temp);
	pcl::copyPointCloud (*keypoints_temp, *keypoints);
	return;
}

/**
     * @brief Detects normals in the input point cloud
     * @param cloud point cloud to be used for normal extraction
     * @param keypoints locations where normals are to be extracted
     * @param normals resulting normals
     * @param radius parameter for setRadiusSearch()
*/
void estimateSurfaceNormals(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normals, float radius)
{
pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts(new pcl::PointCloud<pcl::PointXYZRGB>);
	kpts->points.resize(keypoints->points.size());  
	pcl::copyPointCloud(*keypoints, *kpts);	

// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud (kpts);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (radius);

	// Compute the features
	std::cout << "Computing normals...please wait...";
	ne.compute (*cloud_normals);
	std::cout << "done." << std::endl;

	pcl::copyPointCloud(*cloud_normals, *normals);

	return;
}

/**
     * @brief extract descriptors for given key points
     * @param input point cloud to be used for descriptor extraction
     * @param keypoints locations where descriptors are to be extracted
     * @param normals locations where descriptors are to be extracted
     * @param features resulting descriptors
*/
void extractDescriptors(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normals,  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts(new pcl::PointCloud<pcl::PointXYZRGB>);
	kpts->points.resize(keypoints->points.size());  
	pcl::copyPointCloud(*keypoints, *kpts);	

	// Create the FPFH estimation class, and pass the input dataset+normals to it
	pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud (kpts);
	fpfh.setInputNormals (normals);

	// Output datasets
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	fpfh.setRadiusSearch (25);

	// Compute the features
	std::cout << "Computing FPFH features...please wait...";
	fpfh.compute (*fpfhs);
	std::cout << "done." << std::endl;

	pcl::copyPointCloud(*fpfhs, *features);

	return;
}

/**
     * @brief find the correspondences between source and target images
     * @param source point cloud
     * @param target point cloud
     * @param vector correspondences
*/
void findCorrespondences (pcl::PointCloud<pcl::FPFHSignature33>::Ptr source, pcl::PointCloud<pcl::FPFHSignature33>::Ptr target, std::vector<int>& correspondences)
{
  cout << "correspondence assignment..." << std::flush;
  correspondences.resize (source->size());

  // Use a KdTree to search for the nearest matches in feature space
  pcl::KdTreeFLANN<pcl::FPFHSignature33> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (target);

  // Find the index of the best match for each keypoint, and store it in "correspondences_out"
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);
  for (size_t i = 0; i < source->size (); ++i)
  {
    descriptor_kdtree.nearestKSearch (*source, i, k, k_indices, k_squared_distances);
    correspondences[i] = k_indices[0];
  }
  cout << "OK" << endl;
}

/**
     * @brief filter the correspondences
     * @param vector st
     * @param vector ts
     * @param pointer correspondencesPtr
     * @param vector correspondences
     * @param source keypoint cloud
     * @param target keypoint cloud
*/
void filterCorrespondences(std::vector<int>& st, std::vector<int>& ts, pcl::CorrespondencesPtr correspondencesPtr, std::vector<std::pair<unsigned, unsigned> >& correspondences, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcek, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetk )
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr sk(new pcl::PointCloud<pcl:: PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr tk(new pcl::PointCloud<pcl:: PointXYZI>);
	pcl::copyPointCloud(*sourcek, *sk);
	pcl::copyPointCloud(*targetk, *tk);
	cout << "correspondence rejection..." << std::endl;

	for (unsigned cIdx = 0; cIdx < st.size (); ++cIdx)
		if (ts[st[cIdx]] == cIdx)
			correspondences.push_back(std::make_pair(cIdx, st[cIdx]));

	cout << "Correspondences: "<< correspondences.size() <<endl;
	correspondencesPtr->resize (correspondences.size());
	for (unsigned cIdx = 0; cIdx < correspondences.size(); ++cIdx){
		(*correspondencesPtr)[cIdx].index_query = correspondences[cIdx].first;
		(*correspondencesPtr)[cIdx].index_match = correspondences[cIdx].second;
	}

	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector;
	rejector.setInputCloud(sk);
	rejector.setTargetCloud(tk);
	rejector.setInlierThreshold(2); // <-- aggiunto io 0.4    funziona con 2
	rejector.setInputCorrespondences(correspondencesPtr);
	rejector.getCorrespondences(*correspondencesPtr);

	cout<<"Correspondences after the filter... "<<(*correspondencesPtr).size()<<endl;
	cout << "OK" << endl;

	return;


}

/**
     * @brief determine initial alignment
     * @param pointer correspondencesPtr
     * @param source keypoint cloud
     * @param target keypoint cloud
     * @param source point cloud
     * @param output point cloud
     * @param Matrix 
*/
void determineInitialTransformation (pcl::CorrespondencesPtr correspondencesP, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcek, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetk, pcl::PointCloud<pcl::PointXYZRGB>::Ptr s, pcl::PointCloud<pcl::PointXYZRGB>::Ptr s_out, Eigen::Matrix4f& initial)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr sk(new pcl::PointCloud<pcl:: PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr tk(new pcl::PointCloud<pcl:: PointXYZI>);
	pcl::copyPointCloud(*sourcek, *sk);
	pcl::copyPointCloud(*targetk, *tk);
 	cout << "initial alignment..." << std::flush;
	pcl::registration::TransformationEstimation<pcl::PointXYZI, pcl::PointXYZI>::Ptr transformation_estimation (new pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI>);

	transformation_estimation->estimateRigidTransformation (*sk, *tk, *correspondencesP, initial);

	pcl::transformPointCloud(*s, *s_out, initial);
	cout << "OK" << endl;
	return;
}

/**
     * @brief determine final alignment
     * @param source point cloud
     * @param target point cloud
     * @param output point cloud
     * @param Matrix 
*/
void determineFinalTransformation (pcl::PointCloud<pcl::PointXYZRGB>::Ptr s_alg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr t, pcl::PointCloud<pcl::PointXYZRGB>::Ptr s_out, Eigen::Matrix4f& final)
{
	cout << "final registration..." << endl;
	pcl::Registration<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr registration (new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>);
	registration->setInputCloud(s_alg);
	registration->setInputTarget(t);
	/*registration->setMaxCorrespondenceDistance(4);
	registration->setRANSACOutlierRejectionThreshold (4);
	registration->setTransformationEpsilon (0.000001);
	registration->setMaximumIterations (1000);*/
	cout << "align" << endl;
	registration->align(*s_out);
	cout << "transform" << endl;
	final = registration->getFinalTransformation();
	cout << "OK" << endl;
	return;
}

void determineFinalTransformation_alvise (pcl::PointCloud<pcl::PointXYZRGB>::Ptr s_alg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr t, pcl::PointCloud<pcl::PointXYZRGB>::Ptr s_out, Eigen::Matrix4f& final)
{
	cout << "final registration..." << endl;
	pcl::Registration<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr registration (new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>);
	registration->setInputCloud(s_alg);
	registration->setInputTarget(t);
	/*registration->setMaxCorrespondenceDistance(4);
	registration->setRANSACOutlierRejectionThreshold (4);
	registration->setTransformationEpsilon (0.000001);
	registration->setMaximumIterations (1000);*/
	cout << "align" << endl;
	registration->align(*s_out);
	cout << "transform" << endl;
	final = registration->getFinalTransformation();
	cout << "OK" << endl;
	return;
}

/**
     * @brief determine clusters
     * @param source point cloud
     * @param clusterTolerance
     * @param minCluster
     * @param maxCluster
*/
std::vector<pcl::PointIndices> clusterization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float clusterTolerance, float minCluster, float maxCluster)
{
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (clusterTolerance); 
  ec.setMinClusterSize (minCluster);
  ec.setMaxClusterSize (maxCluster);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);
	return cluster_indices;
}

/**
     * @brief determine if the seal is present, is ok or is not present
     * @param source point cloud
     * @param cluster indices returned by the previous function clusterization()
*/
int sealCheck(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<pcl::PointIndices> cluster_indices)
{
	int check = 0;
	if (cluster_indices.size() == 0)
	{
		check = 1;
	}
	else
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_i (new pcl::PointCloud<pcl::PointXYZRGB>);

		for (size_t i = 0; i < cluster_indices.size (); ++i)
		{
			// Extract the i_th cluster into a new cloud
			pcl::copyPointCloud (*cloud, cluster_indices[i], *cluster_i);
		}

		int cluster_i_points = cluster_i->points.size(), count = 0;

		//Check color
		for(int i=0; i<cluster_i->points.size(); i++){

			if(((float)cluster_i->points[i].r) < 10 && ((float)cluster_i->points[i].g) < 10 && ((float)cluster_i->points[i].b) <10){
				count++;
			}
		}

		if (count > 200){
			check = 2;
		}


	}
	return check;
}

/**
     * @brief Color the dibbles
     * @param source point cloud
     * @param xc, yc, radius parameters for individualize the dibbles
*/
void dibble(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double xc, double yc, double radius)
{
	int check;
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
	if(((cloud->points[i].x - xc)*(cloud->points[i].x - xc) + (cloud->points[i].y - yc)*(cloud->points[i].y - yc)) < (radius*radius))
		{
		cloud->points[i].r = 0;
		cloud->points[i].g = 255;
		cloud->points[i].b = 0;	
		}
	}
	return;
}

/**
     * @brief Color the cable
     * @param cloud with only the cable area
     * @param clod registered
     * @param clod final with the cable colored in red
     * @param cluster_indices
*/
void redCable(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_registered, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_return, std::vector<pcl::PointIndices> cluster_indices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_i)
{
    	for (size_t i = 0; i < cluster_indices.size (); ++i)
      	{
        // Extract the i_th cluster into a new cloud
        pcl::copyPointCloud (*cloud, cluster_indices[i], *cluster_i);

        // Create a unique identifier
        std::stringstream cluster_id ("cluster");
        cluster_id << i;

        }

		for(int i=0; i<cluster_i->points.size(); i++)
		{
				cluster_i->points[i].r = 255;
				cluster_i->points[i].g = 0;
				cluster_i->points[i].b = 0;	
			
		}

		(*cloud_return) = (*cloud_registered) + (*cluster_i);
       		return;
}

bool cableCheck(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable, double minX1, double maxX1, double minX2, double maxX2, double minY1, double maxY1, double minY2, double maxY2)
{
	bool check;
	double maxY, minY;
	int internalPoints = 0;

	if(maxY1 > maxY2)
		maxY = maxY1;
	else
		maxY = maxY2;

	if(minY1 < minY2)
		minY = minY1;
	else
		minY = minY2;

	for(int i=0; i< cable->points.size(); i++){

		//Check if cable is between the dibbles (in y)
		if(cable->points[i].y < maxY && cable->points[i].y > minY){

			//Points between the dibbles (in x)
			if(cable->points[i].x < minX1 && cable->points[i].x > maxX2){
				internalPoints++;
			}

			//Check if cable is over dibble 1
			if(cable->points[i].x > minX1 && cable->points[i].x < maxX1 &&
				cable->points[i].y > minY1 && cable->points[i].y < maxY1){
				internalPoints = 0;
				break;
			}

			//Check if cable is over dibble 2
			if(cable->points[i].x > minX2 && cable->points[i].x < maxX2 &&
			   cable->points[i].y > minY2 && cable->points[i].y < maxY2){
				internalPoints = 0;
				break;
			}
		}
	}

	if(internalPoints > 0 ){
		check = true;
	}
	else{
		check = false;
	}
	

	return check;
}