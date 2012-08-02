/*
 * Homework2.cpp
 *
 *  Created on: May-June 2012
 *      Author: Rossella Petrucci 1020259
*/
#include <vector>
#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_keypoint3D.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/shot_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/gp3.h>
//#include <pcl/surface/marching_cubes_greedy.h>
//#include <pcl/surface/marching_cubes_greedy_dot.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>	// for computing normals with multi-core implementation
#include <pcl/features/fpfh_omp.h>		// for computing FPFH with multi-core implementation
#include <pcl/filters/filter.h>		// for removing NaN from the point cloud

#include <pcl/sample_consensus/sac_model_line.h>

#include <stdlib.h>
#include <cstring>
#include <time.h>
#include <fstream>

using namespace std;

/**
     * @brief remove plane 
     * @param source the input point cloud
     * @param segmented the resulting segmented point cloud
*/
void segmentation (pcl::PointCloud<pcl::PointXYZRGB>::Ptr source, pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented)
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
  seg.setDistanceThreshold (1.2);

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
    pcl::console::print_info ("Subtracted %zu points along the detected plane\n", n - segmented->size ());
// Visualize the result
    pcl::console::print_info ("Segmentend Point cloud... Close window to exit.\n");
    pcl::visualization::PCLVisualizer vis;

    vis.addPointCloud (segmented);
    vis.resetCamera ();
    vis.spin ();
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

int main(int argc, char ** argv)
{
	if (argc < 3) 
	{
		pcl::console::print_info ("Syntax is:  %s source.pcd target.pcd \n", argv[0]);
		return (1);
	}

	// Gestione parametri di input

	/*pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh_);
	pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh_);
	pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh_);*/

 	// Load the input file
 	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
 	pcl::io::loadPCDFile (argv[1], *source_cloud);
 	pcl::console::print_info ("Loaded %s (%zu points)\n", argv[1], source_cloud->size ());

 	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
 	pcl::io::loadPCDFile (argv[2], *target_cloud);
 	pcl::console::print_info ("Loaded %s (%zu points)\n", argv[2], target_cloud->size ());

 	


 	// 								**** REMOVE THE PLANE ****
	//1.Cloud Segmentation
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg_source_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);  
	pcl::console::print_info ("Source Cloud Segmentation\n");
	segmentation(source_cloud, seg_source_cloud);


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg_target_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);  
	pcl::console::print_info ("Target Cloud Segmentation\n");
	segmentation(target_cloud, seg_target_cloud);

	


	//								**** 2.REGISTRATION ****
	//Estimate keypoints
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_keypoints (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_keypoints (new pcl::PointCloud<pcl::PointXYZRGB>);

	detect_keypoints(seg_source_cloud, source_keypoints, 0.01, 3, 2, 0.0);
	pcl::console::print_info ("Detected %zu keypoints\n", source_keypoints->size());

	detect_keypoints(seg_target_cloud, target_keypoints, 0.01, 3, 2, 0.0);
	pcl::console::print_info ("Detected %zu keypoints\n", target_keypoints->size());

	//Estimate surface normals
	pcl::PointCloud<pcl::Normal>::Ptr normals_source_cloud (new pcl::PointCloud<pcl::Normal> ());
	pcl::PointCloud<pcl::Normal>::Ptr normals_target_cloud (new pcl::PointCloud<pcl::Normal> ());
	estimateSurfaceNormals(seg_source_cloud, source_keypoints, normals_source_cloud, 20.0);
	estimateSurfaceNormals(seg_target_cloud, target_keypoints, normals_target_cloud, 20.0);

	//Estimate features
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr f_source_cloud (new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr f_target_cloud (new pcl::PointCloud<pcl::FPFHSignature33>);
	extractDescriptors(seg_source_cloud, source_keypoints, normals_source_cloud, f_source_cloud);
	extractDescriptors(seg_target_cloud, target_keypoints, normals_target_cloud, f_target_cloud);

	//Find and Filter Correspondences
	std::vector<int> source2target; //from source to target
	std::vector<int> target2source; //from target to source
	findCorrespondences(f_source_cloud, f_target_cloud, source2target);
	findCorrespondences(f_target_cloud, f_source_cloud, target2source);
	pcl::CorrespondencesPtr correspondences_Ptr (new pcl::Correspondences);
	std::vector<std::pair<unsigned, unsigned> > correspondences;
	filterCorrespondences(source2target, target2source, correspondences_Ptr, correspondences, source_keypoints, target_keypoints);
	
	//Determine Initial Alignment
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_trasformed (new pcl::PointCloud<pcl::PointXYZRGB>);
	Eigen::Matrix4f initial_transformation_matrix;
	determineInitialTransformation(correspondences_Ptr, source_keypoints, target_keypoints, seg_source_cloud, source_trasformed, initial_transformation_matrix);

	//Determine Final Alignment
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_registered (new pcl::PointCloud<pcl::PointXYZRGB>);

	Eigen::Matrix4f transformation_matrix;
	clock_t start,end;
	double time;

	// 										*****FINAL REGISTRATION*****
	start = clock();
	determineFinalTransformation(source_trasformed, target_cloud, source_registered, transformation_matrix);
	end = clock();

	time = ((double)(end-start))/CLOCKS_PER_SEC;
	cout << "Elapsed time for final registration: " << time << endl;

	//Visualize the Final Registration
	pcl::visualization::PCLVisualizer viewer;
	int v1(0);
	viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
	viewer.setBackgroundColor (0.4, 0.4, 0.4, v1);
	viewer.addCoordinateSystem (0.1, v1);
	viewer.addText ("target_segmented", 10, 10, "v1 text", v1);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(seg_target_cloud);
        viewer.addPointCloud<pcl::PointXYZRGB> (seg_target_cloud, rgb1, "input_cloud_1",v1);
	int v2(0);
	viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
	viewer.setBackgroundColor (0.5, 0.5, 0.5, v2);
	viewer.addCoordinateSystem (0.1, v2);
	viewer.addText ("source_registered", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(source_registered);
	viewer.addPointCloud<pcl::PointXYZRGB> (source_registered, rgb2, "input_cloud_2",v2);
	viewer.resetCamera();
	std::cout << "Registration... Close Window to Exit "<< std::endl;
	while (!viewer.wasStopped ())
	{
		viewer.spin ();
	}

 //3.Determine and color dibbles and cable
	//CLUSTERIZATION
	//Reduce point cloud to the only area where is the cable
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZRGB>);
	for(int i=0; i<source_registered->points.size(); i++){
	if(source_registered->points[i].y>60 && source_registered->points[i].y<78 && source_registered->points[i].x<-8 && source_registered->points[i].x>-32 && source_registered->points[i].z < 346 && source_registered->points[i].z > 336)
	{
		        pcl::PointXYZRGB *p= new pcl::PointXYZRGB();
			p->x= source_registered->points[i].x;
			p->y= source_registered->points[i].y;
			p->z= source_registered->points[i].z;
			p->r= source_registered->points[i].r;
			p->g= source_registered->points[i].g;
			p->b= source_registered->points[i].b;
			temp->points.push_back(*p);
	}}
	std::vector<pcl::PointIndices> cluster_indices_cavo;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cable (new pcl::PointCloud<pcl::PointXYZRGB>);
	cluster_indices_cavo = clusterization(temp, 0.7, 800, 2000);
	redCable(temp, source_registered, cloud_final, cluster_indices_cavo, cluster_cable);	
	//Search for dibbles
        dibble(cloud_final, -22, 74, 0.7);
        dibble(cloud_final, -27, 74.2, 0.7);
	// Visualize the TOTAL result
        pcl::console::print_info ("Dibbles and Cable... Close window to exit.\n");
        pcl::visualization::PCLVisualizer vis;
        vis.addPointCloud (cloud_final);
        vis.resetCamera ();
        vis.spin ();

	//Check if the cable is between the dibbles or not
	bool check_c;
	check_c = cableCheck(cloud_final, cluster_cable, -22.35, -21.65, -27.35, -26.65, 73.65, 74.35, 73.85, 74.55);
	switch(check_c)
	{
		case true:
		cout << "YES: Cable correct" << endl;
		break;
		case false:
		cout << "NO: Cable is not correct" <<endl;
		break;
	}
	
	//Search for the seal
        std::vector<pcl::PointIndices> cluster_indices_seal;
	cluster_indices_seal = clusterization(source_registered, 1, 15000, 17500);
	int check_s;
	check_s = sealCheck(source_registered, cluster_indices_seal);
	switch(check_s)
	{
		case 0:
		cout << "Seal is OK" << endl;
		break;
		case 1:
		cout << "Seal is not correct" <<endl;
		break;
		case 2:
		cout << "Seal is not present" <<endl;
		break;
	}
		
	return 0;	
}