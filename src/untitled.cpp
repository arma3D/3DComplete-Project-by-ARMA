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
	determineInitialTransformation(correspondences_Ptr, source_keypoints, target_keypoints, pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg_source_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_trasformed, initial_transformation_matrix);

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