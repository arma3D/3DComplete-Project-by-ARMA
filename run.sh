#! /bin/bash

./progetto Dataset\ Homework\ PCL/correct01/inliers.pcd Dataset\ Homework\ PCL/correct02/inliers.pcd \
-v \
--voxel-size 							0.1		\
--table-dist 							1.8 	\
--normal-radius 						0.02	\
--feature-radius 						0.02	\
--min-sample-distance 					0.05	\
--max-correspondence-distance 			0.1	 	\
--nr-iterations 						500	\


# ./progetto Dataset\ Homework\ PCL/correct01/inliers.pcd Dataset\ Homework\ PCL/correct02/inliers.pcd \
# -v \
# --voxel-size 							0.4		\
# --table-dist 							1.8 	\
# --normal-radius 						0.001		\
# --feature-radius 						0.001		\
# --min-sample-distance 					0.5		\
# --max-correspondence-distance 			10	 	\
# --nr-iterations 						400		\