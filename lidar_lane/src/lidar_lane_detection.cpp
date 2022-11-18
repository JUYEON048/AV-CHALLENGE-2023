#include <iostream>
#include <pcl/pcl_config.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include<lidar_lane/Line.h>

//$rosrun tf static_transform_publisher 0 0 0 0 0 0 1 velodyne 10

ros::Publisher pub_cluster_result;
ros::Publisher pub_lane_state;
pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_data (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data_xyz (new pcl::PointCloud<pcl::PointXYZ>);
lidar_lane::Line lane_state;


float pre_y_sum = 0.0;
int cloud_cluster_y_sum = 0;

void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *pcl_data);

	int count =0;
	for(int i=0; i< pcl_data->points.size(); i++){
		if(fabs(pcl_data->points[i].x) < 1 && pcl_data->points[i].y > 5 && pcl_data->points[i].y < 10 && pcl_data->points[i].intensity < 20){
			count ++;
		}
	}
	std::cout << count << std::endl;

	float gain = 0.0; 


	pcl::PassThrough<pcl::PointXYZI> PassTh_y;
	PassTh_y.setInputCloud(pcl_data);		// input data
	PassTh_y.setFilterFieldName("y");		// axis
	PassTh_y.setFilterLimits(-3.5, 6.5);	// limit range
	PassTh_y.filter(*pcl_data);				// through filter

	pcl::PassThrough<pcl::PointXYZI> PassTh_x;
	PassTh_x.setInputCloud(pcl_data);		// input data
	PassTh_x.setFilterFieldName("x");		// axis
	PassTh_x.setFilterLimits(5, 25);		// limit range
	PassTh_x.filter(*pcl_data);	

	pcl::PassThrough<pcl::PointXYZI> PassTh_z;
	PassTh_z.setInputCloud(pcl_data);
	PassTh_z.setFilterFieldName("z");
	PassTh_z.setFilterLimits(-4.0, -1.5);	//(-10.0, -1.5)
	PassTh_z.filter(*pcl_data); 

	pcl::PassThrough<pcl::PointXYZI> PassTh_intensity;
	PassTh_intensity.setInputCloud(pcl_data);
	PassTh_intensity.setFilterFieldName("intensity");
	PassTh_intensity.setFilterLimits(50, 200); //(50,200) 65
	PassTh_intensity.filter(*pcl_data);

  	pcl::copyPointCloud(*pcl_data, *pcl_data_xyz);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals (true); 	//optional
	mls.setInputCloud (pcl_data_xyz);
	mls.setPolynomialOrder (2); 	//5 //Get the order of the polynomial to be fit. 
	mls.setSearchMethod (tree);
	mls.setSearchRadius (0.1); 		// Set the sphere radius that is to be used for determining the k-nearest neighbors used for fitting. 
	mls.setPolynomialFit(true); 	//If true, the surface and normal are approximated using a polynomial estimation
									// (if false, only a tangent one).
	mls.process(*smoothedCloud);
	pcl::copyPointCloud(*smoothedCloud, *pcl_data_xyz);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
  	tree2->setInputCloud (pcl_data_xyz);  //KdTree 생성 
	std::vector<pcl::PointIndices> cluster_indices; // 군집화된 결과물의 Index 저장, 다중 군집화 객체는 cluster_indices[0] 순으로 저장 
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setInputCloud (pcl_data_xyz);     	// 입력   
	ec.setClusterTolerance (0.3);  			// 2cm  0.02  || 0.5
	ec.setMinClusterSize (2);     			// 최소 포인트 수 100
	ec.setMaxClusterSize (70);   			// 최대 포인트 수 25000
	ec.setSearchMethod (tree2);      		// 위에서 정의한 탐색 방법 지정 
	ec.extract (cluster_indices);   		// 군집화 적용 

	float cloud_cluster_y_negative = 0.0;
	float cloud_cluster_y_positive = 0.0;

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		for(int i = 0; i < pcl_data_xyz->points.size(); i++)
		{			
			if(3.0 <= pcl_data_xyz->points[i].y || pcl_data_xyz->points[i].y <= -3.5)
			{
				continue;
			}
			else
			{	
				if(pcl_data_xyz->points[i].x >= 8)
				{
					if(cloud_cluster_y_negative == 0.0)
					{
						if(pcl_data_xyz->points[i].y < 0){
							cloud_cluster_y_negative = pcl_data_xyz->points[i].y;
						}
					}

					else if(cloud_cluster_y_positive == 0.0)
					{
						if(pcl_data_xyz->points[i].y > 0){
							cloud_cluster_y_positive = pcl_data_xyz->points[i].y;
						}
					}

					else 
					{
						if (pcl_data_xyz->points[i].y < 0) //Right 
						{
							if(pcl_data_xyz->points[i].y > cloud_cluster_y_negative)
								cloud_cluster_y_negative = pcl_data_xyz->points[i].y; 
						}

						else if (pcl_data_xyz->points[i].y > 0) //Left
						{ 
							if(pcl_data_xyz->points[i].y < cloud_cluster_y_positive)
								cloud_cluster_y_positive = pcl_data_xyz->points[i].y; 
						}
					}
				}
			}			
		}
	}

	
	// for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	// {
	// 	for(int i = 0; i < pcl_data_xyz->points.size(); i++)
	// 	{
	// 		if(i==0){
	// 			if(pcl_data_xyz->points[i].y < 0){
	// 				cloud_cluster_y_negative = pcl_data_xyz->points[i].y;
	// 			}
	// 			else{
	// 				cloud_cluster_y_positive = pcl_data_xyz->points[i].y;
	// 			}
	// 			std::cout << "negative : " << cloud_cluster_y_negative << ", " << "postive : " << cloud_cluster_y_positive << std::endl;
	// 		}

	// 		else{

	// 			if( 5.0 <= pcl_data_xyz->points[i].y || pcl_data_xyz->points[i].y <= -3.5)
	// 			{
	// 				continue;
	// 			}
	// 			else
	// 			{	
	// 				if(pcl_data_xyz->points[i].x >= 8)
	// 				{
	// 					// std::cout << pcl_data_xyz->points[i].y << std::endl;

	// 					if (pcl_data_xyz->points[i].y < 0) //Right 
	// 					{
	// 						if(pcl_data_xyz->points[i].y > cloud_cluster_y_negative)
	// 							cloud_cluster_y_negative = pcl_data_xyz->points[i].y; 
	// 					}

	// 					if (pcl_data_xyz->points[i].y > 0) //Left
	// 					{ 
	// 						if(pcl_data_xyz->points[i].y < cloud_cluster_y_positive)
	// 							cloud_cluster_y_positive = pcl_data_xyz->points[i].y; 
	// 					}
	// 				}

	// 			}
	// 		}
	// 		//std::cout << "x ************** " << pcl_data_xyz->points[i].x << std::endl;
	// 		//std::cout << "y ************** " << (*cloud_cluster)[i].y << std::endl << std::endl;	
	// 		//std::cout << "z ************** " << (*cloud_cluster)[i].z << std::endl;
	// 	}
	// 	//std::cout << "cloud_cluster print ---> " << *(pcl_data_xyz)->points.data() << std::endl;
	// }
	/*if(cloud_cluster_y_negative == -10 || cloud_cluster_y_positive == 10)
	{
		cloud_cluster_y_sum = cloud_cluster_y_sum; //continue?
	}
	else
	{
		cloud_cluster_y_sum = 0;
		cloud_cluster_y_sum = cloud_cluster_y_negative + cloud_cluster_y_positive;
	}*/
	if(count < 500){
		gain = -1.1;
	}
	else if(count < 1000){
		gain = -4.5;
	}
	else{
		gain = -8;
	}

	pre_y_sum = cloud_cluster_y_sum;
	cloud_cluster_y_sum = (cloud_cluster_y_negative + cloud_cluster_y_positive); //* gain;

	if(fabs(cloud_cluster_y_sum) > 30)
	{
		if(cloud_cluster_y_sum > 0)
		{
			cloud_cluster_y_sum = 30;
		}
		else
		{
			cloud_cluster_y_sum = -30;
		}
	}
	std::cout << "cur - pre y_sum : " << cloud_cluster_y_sum - pre_y_sum << std::endl;

	std::cout << "R : "<< cloud_cluster_y_negative << ", L : " << cloud_cluster_y_positive << std::endl;
	std::cout << "sum : " << cloud_cluster_y_sum << std::endl;
	std::cout << "==================" << std::endl;

    lane_state.header.stamp = ros::Time::now(); 
    
	// if(cloud_cluster_y_sum >= 4)
	// {
	// 	lane_state.data = cloud_cluster_y_sum  * - 4;
	// }
	// else
	// {
	// 	lane_state.data = cloud_cluster_y_sum * - 2;
	// }
	
	

	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*pcl_data_xyz, output);
	pub_cluster_result.publish(output);
	pub_lane_state.publish(lane_state);
}



int main (int argc, char** argv)
{
	ros::init(argc, argv, "lidar_lane_detection");
	ros::NodeHandle nh;
	pub_cluster_result = nh.advertise<sensor_msgs::PointCloud2>("lidar_lane",1);
	pub_lane_state = nh.advertise<lidar_lane::Line>("lane_state_lidar",1);
	ros::Subscriber sub = nh.subscribe("Combined_velo", 1, callback);
	ros::spin();
}
