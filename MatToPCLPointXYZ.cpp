pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudSpeed::ConverDepthTo3D_PCL_Fast(const cv::Mat& depthmap){
	// for converting a depthmap to 3D cartesian coordinates
	// FOR SPEEDING UP THE CODE 
	int dl = 50;//dl = the length of squares in the corners		
	pcl::PointCloud<pcl::PointXYZ>::Ptr  point_cloud_ptr (new  pcl::PointCloud<pcl::PointXYZ>);
	point_cloud_ptr->points.reserve(DEPTH_WIDTH*DEPTH_HEIGHT- dl*dl*4);//for speeding up code
	//point_cloud_ptr->points.resize(point_cloud_ptr->points.size());
	pcl::PointXYZ point; 
	float Z;

	//int cnt = 0;
	double iMinDepthCyMultiInversY = -INVERSE_FOCAL_LENGTH_Y* DEPTH_CY;
	for (int i=0; i<(DEPTH_HEIGHT); i++){
			for (int j=0; j<DEPTH_WIDTH; j++){
				//square surfaces at the corners deleting
				/*important deleting this point_cloud_ptr need to delete -dl*dl*4 otherwice add this */
				//mindis is called most of the time
				/*pass-through filter that filter 40 cm until 3.5 m */
				Z = depthmap.at<float>((i-DEPTH_CY),j);
				if (MINDIS < Z &&(((DEPTH_WIDTH - dl) > j) || ((DEPTH_HEIGHT - dl)>i)) && 
					(((dl < j) || (dl < i))) && (((DEPTH_WIDTH - dl) > j) || 
					(dl < i)) && (((DEPTH_HEIGHT - dl) > i) || (dl < j))&& MAXDIS > Z){ 
			
						//point_cloud_ptr -> points[cnt].x = (j - DEPTH_CX) * Z  * INVERSE_FOCAL_LENGTH_X;
						//point_cloud_ptr -> points[cnt].y = iMinDepthCyMultiInversY * Z  ;
						//point_cloud_ptr -> points[cnt].z = Z;

						point.x = (j - DEPTH_CX) * Z  * INVERSE_FOCAL_LENGTH_X;
						point.y = iMinDepthCyMultiInversY * Z  ;
						point.z = Z;

						point_cloud_ptr -> points.push_back(point);
						//cnt++;
						//kan nog tijd gewonnen worden door pushback functie te veranderen naar pointcloud->point[cnt].z = Z methode	
						
				}
			}
			iMinDepthCyMultiInversY +=INVERSE_FOCAL_LENGTH_Y;
		}
		
		point_cloud_ptr -> width = (int)point_cloud_ptr->points.size();
		point_cloud_ptr -> height = 1;

	
	return point_cloud_ptr;
}