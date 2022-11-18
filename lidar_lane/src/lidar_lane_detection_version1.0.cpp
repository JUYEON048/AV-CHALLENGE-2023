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
#include <pcl/filters/radius_outlier_removal.h>

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_data (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data_xyz (new pcl::PointCloud<pcl::PointXYZ>);

void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *pcl_data);

	pcl::PassThrough<pcl::PointXYZI> PassTh_y;
	PassTh_y.setInputCloud(pcl_data);		// input data
	PassTh_y.setFilterFieldName("y");		// axis
	PassTh_y.setFilterLimits(-3.5, 6.5);	// limit range
	PassTh_y.filter(*pcl_data);				// through filter

	pcl::PassThrough<pcl::PointXYZI> PassTh_z;
	PassTh_z.setInputCloud(pcl_data);
	PassTh_z.setFilterFieldName("z");
	PassTh_z.setFilterLimits(-4.0, -1.5);	//(-10.0, -1.5)
	PassTh_z.filter(*pcl_data); 

	pcl::PassThrough<pcl::PointXYZI> PassTh_intensity;
	PassTh_intensity.setInputCloud(pcl_data);
	PassTh_intensity.setFilterFieldName("intensity");
	PassTh_intensity.setFilterLimits(70, 200); //(50,200)
	PassTh_intensity.filter(*pcl_data); 

  	pcl::copyPointCloud(*pcl_data, *pcl_data_xyz);

	/*#include <pcl/filters/statistical_outlier_removal.h>
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (pcl_data_xyz); 
	sor.setMeanK (50); 
	sor.setStddevMulThresh (1.0); 
	sor.filter (*pcl_data_xyz);  */

	/*pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	outrem.setInputCloud(pcl_data_xyz);
	outrem.setRadiusSearch(0.01);
	outrem.setMinNeighborsInRadius(15); 
	outrem.filter (*pcl_data_xyz);*/


	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals (true); //optional
	mls.setInputCloud (pcl_data_xyz);
	//mls.setPolynomialOrder (2); //5 //Get the order of the polynomial to be fit. 
	mls.setPolynomialOrder (2); //5 //Get the order of the polynomial to be fit. 
	mls.setSearchMethod (tree);
	//mls.setSearchRadius (0.06); //0.05 // Set the sphere radius that is to be used for determining the k-nearest neighbors used for fitting. 
	mls.setSearchRadius (0.1); //0.05 // Set the sphere radius that is to be used for determining the k-nearest neighbors used for fitting. 
	mls.setPolynomialFit(true); //If true, the surface and normal are approximated using a polynomial estimation
									// (if false, only a tangent one).
	mls.process(*smoothedCloud);


	for(size_t i = 0; i< smoothedCloud->points.size(); ++i)
	{
		std::cout << "x: " << smoothedCloud -> points[i].x << std::endl;
		std::cout << "y: " << smoothedCloud -> points[i].y << std::endl;
	}







	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*pcl_data_xyz, output);
	pub.publish(output);
}



int main (int argc, char** argv)
{
	ros::init(argc, argv, "lidar_line_detection");
	ros::NodeHandle nh;
	pub = nh.advertise<sensor_msgs::PointCloud2>("lidar_line",1);
	ros::Subscriber sub = nh.subscribe("Combined_velo", 1, callback);
	ros::spin();
}
