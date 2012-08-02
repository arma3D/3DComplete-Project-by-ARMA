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

 	// vettore che conterrà le nuvole da allineare
	vector<DetailedCloud> object_templates;
	object_templates.resize(0);


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

	cout  << endl << "**** ALIGNMENT ****" << endl << endl;
	// inserimento target sull'array
	DetailedCloud daAllineare(normal_radius, feature_radius);
	daAllineare.setInputCloud(seg_target_cloud);
	object_templates.push_back(daAllineare);

	// Assign to the target DetailedCloud
	DetailedCloud target_detailed_cloud;
	target_detailed_cloud.setInputCloud(seg_source_cloud);

	// Set the TemplateAlignment inputs
	TemplateAlignment template_align(min_sample_distance, max_correspondence_distance, nr_iterations);
	for (size_t i = 0; i < object_templates.size (); ++i)
	{
		template_align.addTemplateCloud(object_templates[i]);
	}
	template_align.setTargetCloud(target_detailed_cloud);

	// Find the best template alignment
	cout << "find alignment" << endl;

	TemplateAlignment::Result best_alignment;
	int best_index = template_align.findBestAlignment(best_alignment);
	const DetailedCloud &best_template = object_templates[best_index];
	cout << "end alignment" << endl;

	// Print the alignment fitness score (values less than 0.00002 are good)
	printf("Best fitness score: %f\n", best_alignment.fitness_score);

	// Print the rotation matrix and translation vector
	Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
	Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);

	printf ("\n");
	printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
	printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
	printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
	printf ("\n");
	printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

	// Save the aligned template for visualization
	pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;

	pcl::transformPointCloud(*best_template.getPointCloud(), transformed_cloud, best_alignment.final_transformation);
	pcl::io::savePCDFileBinary("output.pcd", transformed_cloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_pointer = transformed_cloud.makeShared();

	pcl::visualization::PCLVisualizer viewerAligned;
	visualizeTwo(&viewerAligned, "source", "target aligned", seg_source_cloud, transformed_cloud_pointer);
	while(!viewerAligned.wasStopped())
	{
		viewerAligned.spin();
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr unione (new pcl::PointCloud<pcl::PointXYZRGB>); 
	(*unione) = transformed_cloud + (*seg_source_cloud);
	
	pcl::visualization::PCLVisualizer vis;
	vis.addPointCloud(unione);
	vis.resetCamera();
	vis.spin();

	return 0;	
}