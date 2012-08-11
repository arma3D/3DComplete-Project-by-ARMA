#include <vector>
#include <string>
#include <sstream>
#include <pcl/point_types.h>
#include <stdlib.h>
#include <cstring>
#include <time.h>
#include <fstream>
#include <pcl/console/parse.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include "segmentation_plane.h"

#define TABLE_TH 		1.0
#define ANT_TH 			0.06
#define COPLANARITY_TH	1
#define _USE_MATH_DEFINES

using namespace std;

bool coplanarity(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr onlyAnt, bool stamp, float co_th = COPLANARITY_TH)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr table(new pcl::PointCloud<pcl::PointXYZRGB>);  
	pcl::ModelCoefficients::Ptr table_coeff(new pcl::ModelCoefficients);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ant(new pcl::PointCloud<pcl::PointXYZRGB>);  
	pcl::ModelCoefficients::Ptr ant_coeff(new pcl::ModelCoefficients);

	get_a_plane(cloud, table, TABLE_TH, false, table_coeff);
	get_a_plane(onlyAnt, ant, ANT_TH, true, ant_coeff);

	float a = table_coeff->values[0];
	float b = table_coeff->values[1];
	float c = table_coeff->values[2];
	float d = table_coeff->values[3];

	float a_ = ant_coeff->values[0];
	float b_ = ant_coeff->values[1];
	float c_ = ant_coeff->values[2];
	float d_ = ant_coeff->values[3];

	if(stamp)
	{
		cout << "table__ coefficients: a: " 
			<< a << " b: " << b << " c: " << c << " d: " << d << endl;
		cout << "antenna coefficients: a: "
			<< a_ << " b: " << b_ << " c: " << c_ << " d: " << d_ << endl;
	}

	float diff = abs(a-a_) + abs(b-b_) + abs(c-c_);

	float scalar_product = (a*a_) + (b*b_) + (c*c_);
	float mod_1 = sqrt(pow(a,2) + pow(b,2) + pow(c,2));
	float mod_2 = sqrt(pow(a_,2) + pow(b_,2) + pow(c_,2));

	float alfa = acos((scalar_product) / (mod_1 * mod_2));
	alfa = alfa*(180/M_PI);

	cout << "differenza: " << diff << " delta angle: " << alfa << endl;

	if(alfa <= co_th)
		return true;
	else
		return false;
}

bool coplanarity(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr onlyAnt, float distanceThreshold_table, float distanceThreshold_ant, bool stamp, float co_th = COPLANARITY_TH)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr table(new pcl::PointCloud<pcl::PointXYZRGB>);  
	pcl::ModelCoefficients::Ptr table_coeff(new pcl::ModelCoefficients);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ant(new pcl::PointCloud<pcl::PointXYZRGB>);  
	pcl::ModelCoefficients::Ptr ant_coeff(new pcl::ModelCoefficients);

	get_a_plane(cloud, table, distanceThreshold_table, false, table_coeff);
	get_a_plane(onlyAnt, ant, distanceThreshold_ant, false, ant_coeff);

	float a = table_coeff->values[0];
	float b = table_coeff->values[1];
	float c = table_coeff->values[2];
	float d = table_coeff->values[3];

	float a_ = ant_coeff->values[0];
	float b_ = ant_coeff->values[1];
	float c_ = ant_coeff->values[2];
	float d_ = ant_coeff->values[3];

	if(stamp)
	{
		cout << "table__ coefficients: a: " 
			<< a << " b: " << b << " c: " << c << " d: " << d << endl;
		cout << "antenna coefficients: a: "
			<< a_ << " b: " << b_ << " c: " << c_ << " d: " << d_ << endl;
	}

	float diff = abs(a-a_) + abs(b-b_) + abs(c-c_);

	float scalar_product = (a*a_) + (b*b_) + (c*c_);
	float mod_1 = sqrt(pow(a,2) + pow(b,2) + pow(c,2));
	float mod_2 = sqrt(pow(a_,2) + pow(b_,2) + pow(c_,2));

	float alfa = acos((scalar_product) / (mod_1 * mod_2));
	alfa = alfa*(180/M_PI);

	cout << "differenza: " << diff << " delta angle: " << alfa << endl;

	if(alfa <= co_th)
		return true;
	else
		return false;	
}