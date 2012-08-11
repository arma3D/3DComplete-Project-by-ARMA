/*
 * codice_progetto.cpp
 *
 *  Created on: May-June 2012
 *      Author: Rossella Petrucci 	1020259
 *		Author: Alvise Rigo 		1013970
 *		Author: Albero Rubin
*/

#include <vector>
#include <string>
#include <sstream>
#include <pcl/point_types.h>
#include <stdlib.h>
#include <cstring>
#include <time.h>
#include <fstream>
#include <pcl/console/parse.h>
#include "include/funzioni_progetto.h"
#include <pcl/registration/registration.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/registration/icp.h>


using namespace std;

int main(int argc, char ** argv)
{
	if (argc < 3 || pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help")) 
	{
		pcl::console::print_info ("Syntax is:  %s source.pcd target.pcd \n", argv[0]);
		pcl::console::print_info ("Optional: \n");
		pcl::console::print_info ("VOXELING: \n ");
		pcl::console::print_info ("-v				-> enable voxellization \n ");
		pcl::console::print_info ("-visualizeVoxel	-> visualize voxel result \n ");
		pcl::console::print_info ("--voxel-size \n");
		pcl::console::print_info ("--table-dist \n ");
		pcl::console::print_info ("FEATURE: \n ");
		pcl::console::print_info ("--normal-radius \n ");
		pcl::console::print_info ("--feature-radius \n ");
		pcl::console::print_info ("ALIGNMENT: \n ");
		pcl::console::print_info ("--min-sample-distance \n ");
		pcl::console::print_info ("--max-correspondence-distance \n ");
		pcl::console::print_info ("--nr-iterations \n ");
		return (1);
	}

	bool voxel = false;
	bool visualizeVoxel = false;
	float voxel_grid_size  = 0.05f;
	float distanceThreshold = 1.8f;

	float normal_radius = 0.02f;
	float feature_radius = 0.02f;

	float min_sample_distance = 0.05f;
	float max_correspondence_distance = 0.01f*0.01f;
	float nr_iterations = 200;

	if (pcl::console::find_switch (argc, argv, "-v")) voxel = true;

	if (pcl::console::find_switch (argc, argv, "--visualizeVoxel")) visualizeVoxel = true;

	pcl::console::parse_argument (argc, argv, "--voxel-size",  voxel_grid_size);
	cout << "voxel grid size: " << voxel_grid_size << endl;

	pcl::console::parse_argument (argc, argv, "--table-dist",  distanceThreshold);
	cout << "table segmentation distance: " << distanceThreshold << endl;

	pcl::console::parse_argument (argc, argv, "--normal-radius",  normal_radius);
	cout << "normal radius: " << normal_radius << endl;

	pcl::console::parse_argument (argc, argv, "--feature-radius",  feature_radius);
	cout << "feature radius: " << feature_radius << endl;

	pcl::console::parse_argument (argc, argv, "--min-sample-distance",  min_sample_distance);
	cout << "minimum sample distance: " << min_sample_distance << endl;

	pcl::console::parse_argument (argc, argv, "--max-correspondence-distance",  max_correspondence_distance);
	cout << "max correspondence distance: " << max_correspondence_distance << endl;

	pcl::console::parse_argument (argc, argv, "--nr-iterations",  nr_iterations);
	cout << "number of iterations: " << nr_iterations << endl;

	if (voxel)
	{
		cout << "...:::voxelling enabled:::..." << endl;
	}
	
	
/* ################################################################################################################## */




	// Load the input file
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile(argv[1], *source_cloud);
	pcl::console::print_info ("Loaded %s (%zu points)\n", argv[1], source_cloud->size ());

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile (argv[2], *target_cloud);
	pcl::console::print_info ("Loaded %s (%zu points)\n", argv[2], target_cloud->size ());


 	// 								**** REMOVE THE PLANE ****
	//1.Cloud Segmentation
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg_source_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);  
	pcl::console::print_info ("Source Cloud Segmentation\n");
	segmentation(source_cloud, seg_source_cloud, distanceThreshold, false);


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg_target_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);  
	pcl::console::print_info ("Target Cloud Segmentation\n");
	segmentation(target_cloud, seg_target_cloud, distanceThreshold, false);


//										**** VOXELLING ****
	if (voxel)
	{
		cout  << endl << "**** VOXELLING ****" << endl << endl;
		cout << "total points source cloud before voxelling: " << seg_source_cloud->size() << endl;
		cout << "total points target cloud before voxelling: " << seg_target_cloud->size() << endl;

		doVoxel(seg_source_cloud, voxel_grid_size);
		cout << "total points source cloud after voxelling: " << seg_source_cloud->size() << endl;

		doVoxel(seg_target_cloud, voxel_grid_size);
		cout << "total points target cloud after voxelling: " << seg_target_cloud->size() << endl;

		if(visualizeVoxel)
		{
			pcl::visualization::PCLVisualizer viewer;
			visualizeTwo(&viewer, "seg source voxelled", "seg target voxelled", seg_source_cloud, seg_target_cloud);
			while (!viewer.wasStopped())
			{
				viewer.spin();
			}
		}
	}
	
//										**** ALIGNMENT ****

	
	/*pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> ndt;
	// Setting scale dependent NDT parameters
	// Setting minimum transformation difference for termination condition.
	ndt.setTransformationEpsilon(0.00001);
	// Setting maximum step size for More-Thuente line search.
	ndt.setStepSize(0.00001);
	//Setting Resolution of NDT grid structure (VoxelGridCovariance).
	ndt.setResolution(200);

	// Setting max number of registration iterations.
	ndt.setMaximumIterations(35);

	ndt.setInputCloud(seg_source_cloud);
	ndt.setInputTarget(seg_target_cloud);

	// Set initial alignment estimate found using robot odometry.


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cout << "aligning..." << endl;
	ndt.align(*output_cloud);

	std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged() << " score: " << ndt.getFitnessScore() << std::endl;



	// Transforming unfiltered, input cloud using found transform.
	pcl::transformPointCloud(*seg_source_cloud, *output_cloud, ndt.getFinalTransformation ());

	// Saving transformed input cloud.
	pcl::io::savePCDFileASCII("room_scan2_transformed.pcd", *output_cloud);

	*/


pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);

cloud_in = seg_source_cloud;
cloud_out = seg_target_cloud;

 
/*  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZRGB> Final;
  icp.setMaxCorrespondenceDistance(4);
  icp.setMaximumIterations(50);
  icp.setTransformationEpsilon(1e-6);
  clock_t start,end;
  double time;
  start=clock();
	
  icp.align(Final);
  end=clock();
  time=((double)(end-start))/CLOCKS_PER_SEC;

  cout<<"Elapsed time for final registration: "<<time<<endl;
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
*/

	RegistrationObj regObj;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalPtr;
	Eigen::Matrix4f transformationMatrix;

	pcl::visualization::PCLVisualizer viewer;
	visualizeTwo(&viewer, "cloud_in_before_reg", "cloud_out_before_reg", cloud_in, cloud_out);
	while (!viewer.wasStopped())
	{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	regObj.setInputCloud(cloud_in);
	regObj.setTargetCloud(cloud_out);

	cout << "aligning" << endl;

	if(regObj.align(finalPtr))
	{
		//finalPtr = regObj.getRegisteredCloud();
		double timeElapsed = regObj.getTime();
		transformationMatrix = regObj.getTransformation();

		ofstream out("matrix_result");
		regObj.writeResult(&out);

		cout << "registered in " << timeElapsed << " seconds" << endl;
		cout << "transformation matrix:" << endl << transformationMatrix << endl;
		regObj.visualizeResult();
	}

	/*cout << "second transformation" << endl;

	regObj.alignWithMatrix(cloud_out);

	pcl::visualization::PCLVisualizer viewer_2;
	visualizeTwo(&viewer_2, "cloud_in", "cloud_out", cloud_in, cloud_out);*/


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile(argv[1], *source_cloud_2);
	pcl::console::print_info ("Loaded %s (%zu points)\n", argv[1], source_cloud_2->size ());

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile (argv[2], *target_cloud_2);
	pcl::console::print_info ("Loaded %s (%zu points)\n", argv[2], target_cloud_2->size ());


	pcl::visualization::PCLVisualizer viewer_2;
	visualizeTwo(&viewer_2, "cloud_in_before_reg", "cloud_out_before_reg", source_cloud_2, target_cloud_2);
	while (!viewer_2.wasStopped())
	{
			viewer_2.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


	regObj.alignWithMatrix(source_cloud_2);

	pcl::visualization::PCLVisualizer viewer_3;
	visualizeTwo(&viewer_3, "cloud_in_before_reg", "cloud_out_before_reg", source_cloud_2, target_cloud_2);
	while (!viewer_3.wasStopped())
	{
			viewer_3.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;	
}