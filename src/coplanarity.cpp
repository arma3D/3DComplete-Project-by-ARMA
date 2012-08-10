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

#include <pcl/registration/registration.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/registration/icp.h>

 #include "include/funzioni_progetto.h"
 #include "include/funzioni_segmentation.h"


using namespace std;

int main(int argc, char ** argv)
{
	if (argc < 3 || pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help")) 
	{
		pcl::console::print_info ("Syntax is:  %s source.pcd target.pcd \n", argv[0]);
		pcl::console::print_info ("Optional: \n");
		pcl::console::print_info ("--table-dist \n ");
		pcl::console::print_info ("--ant-dist \n ");
		pcl::console::print_info ("--eps-angle \n ");
		return (1);
	}

	float distanceThreshold_table = 1.8f;
	float distanceThreshold_ant = 0.5f;
	float eps_angle = 0.5f;

	pcl::console::parse_argument (argc, argv, "--table-dist",  distanceThreshold_table);
	cout << "table segmentation distance: " << distanceThreshold_table << endl;

	pcl::console::parse_argument (argc, argv, "--ant-dist",  distanceThreshold_ant);
	cout << "antenna segmentation distance: " << distanceThreshold_ant << endl;

	pcl::console::parse_argument (argc, argv, "--eps-angle",  eps_angle);
	cout << "epsilon angle: " << eps_angle << endl;

	
	
/* ################################################################################################################## */




	// Load the input file
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile(argv[1], *cloud);
	pcl::console::print_info("Loaded %s (%zu points)\n", argv[1], cloud->size());


 	// 								**** REMOVE THE PLANE ****
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr no_table_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  
	pcl::console::print_info("Source Cloud Segmentation\n");
	segmentation(cloud, no_table_cloud, distanceThreshold_table, false);

	// 								**** REMOVE THE  ANT ****
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr no_ant_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  
	pcl::console::print_info("Source Cloud Segmentation\n");
	segmentation(cloud, no_ant_cloud, distanceThreshold_ant, false);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);  
	pcl::ModelCoefficients::Ptr plane_coeff (new pcl::ModelCoefficients);

	getPlane(cloud, plane, distanceThreshold_table, false, plane_coeff);

	std::cerr << "Plane-coefficients table: " << plane_coeff->values[0] << " "
		<< plane_coeff->values[1] << " "
		<< plane_coeff->values[2] << " "
		<< plane_coeff->values[3] << std::endl;	

	Eigen::Vector4f planeVect;
	float curvature;
	pcl::computePointNormal(*plane, planeVect, curvature);
	cout << "vettore piano: " << planeVect << endl;

	Eigen::Vector3f vect;
	vect(0) = planeVect(0);
	vect(1) = planeVect(1);
	vect(2) = planeVect(2);

	/*vect(0) = -1;
	vect(1) = 2;
	vect(2) = 30;*/

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ant(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ModelCoefficients::Ptr ant_coeff (new pcl::ModelCoefficients);
	getAnt(no_table_cloud, ant, distanceThreshold_ant, false, vect, eps_angle, ant_coeff);
	
	std::cerr << "Plane-coefficients antenna: " << ant_coeff->values[0] << " "
		<< ant_coeff->values[1] << " "
		<< ant_coeff->values[2] << " "
		<< ant_coeff->values[3] << std::endl;

	Eigen::Vector4f antVect;
	pcl::computePointNormal(*ant, antVect, curvature);

	cout << "plane vect: " << planeVect << endl << "ant vect: " << antVect << endl;

	pcl::visualization::PCLVisualizer viewer;
	visualizeTwo(&viewer, "table", "ant", ant, plane);
	while (!viewer.wasStopped())
	{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}



	int i = 0;
	int j = 0;
	int k = 0;

	cout << "i: " << i << " j: " << j <<" k: " << k << endl;

	float x = 0;
	float y = 0;
	float z = 0;

	float x_plane = 0;
	float y_plane = 0;
	float z_plane = 0;

	float a = ant_coeff->values[0];
	float b = ant_coeff->values[1];
	float c = ant_coeff->values[2];
	float d = ant_coeff->values[3];

	float a_plane = plane_coeff->values[0];
	float b_plane = plane_coeff->values[1];
	float c_plane = plane_coeff->values[2];
	float d_plane = plane_coeff->values[3];

	int iter = 50;
	uint8_t re = 255, gr = 0, bl = 0;
	uint32_t red = ((uint32_t)re << 16 | (uint32_t)gr << 8 | (uint32_t)bl);

	uint8_t rr = 0, gg = 255, bb = 0;
	uint32_t green = ((uint32_t)rr << 16 | (uint32_t)gg << 8 | (uint32_t)bb);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_plane_(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointXYZRGB toAdd;
	pcl::PointXYZRGB toAdd_;

	cout << "prova: " << (((-b-c-d)/a) - ((-b_plane-c_plane-d_plane)/a_plane)) << endl;

	for (i = 1; i < iter; ++i)
	{ // X
		x = (-b*j - c*k - d)/a;
		x_plane = (-(b_plane)*(j*1) - (c_plane)*(k*1) - d_plane)/a_plane;
		toAdd.x = x;
		toAdd_.x = x_plane;

		/*cout << "j: " << j <<" k: " << k << endl;
		cout << "x: " << x << endl;
		cout << "x plane: " << x_plane << endl;*/

		for (j = 1; j < iter/**100*/; ++j)
		{ // Y
			y = (-a*i - c*k - d)/b;
			y_plane = (-a_plane*(i*1) - c_plane*(k*1) - d_plane)/b_plane;
			toAdd.y = y;
			toAdd_.y = y_plane;

			/*cout << "delta_y: " << y - y_plane << endl;*/

			for (k = 1; k < iter/*/8*/; ++k)
			{ // Z
				z = (-a*i - b*j - d)/c;
				z_plane = (-a_plane*(i*1) - b_plane*(j*1) - d_plane)/c_plane;
				/*cout << "delta_z: " << z - z_plane << endl;*/
				
				toAdd.z = z;				
				toAdd_.z = z_plane;

				toAdd.rgb = *reinterpret_cast<float*>(&green);
				toAdd_.rgb = *reinterpret_cast<float*>(&red);
				test_plane->push_back(toAdd);
				test_plane->push_back(toAdd_);
			}
		}
	}

	pcl::visualization::PCLVisualizer vis;
	vis.addPointCloud(test_plane);
	vis.resetCamera();
	vis.spin();
	/*visualizeTwo(&vis, "primo piano", "secondo piano", test_plane, test_plane_);
	vis.spin();*/

	return 0;	
}