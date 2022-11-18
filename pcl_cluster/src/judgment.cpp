#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <iterator>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>

#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "pcl_cluster/Obj_state.h"
#include "pcl_cluster/Obj_hill_state.h"
#include "pcl_cluster/Obj_tracking.h"


//using std::string;
using std::vector;

ros::Publisher pub;
ros::Publisher pub_dist;
ros::Publisher pub_hill_dist;
ros::Publisher pub_steer;
void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    auto cl = (*cloud);
    
    pcl::PointCloud<pcl::PointXYZ> test_cloud;
    pcl::PointCloud<pcl::PointXYZ> test_hill_cloud;

    auto obj_dist = 999;
    auto obj_hill_dist = 999;
    int obj_steer = 0;

    //std_msgs::Float32 obj_state;
    pcl_cluster::Obj_state obj_state;
    pcl_cluster::Obj_hill_state obj_hill_state;
    pcl_cluster::Obj_tracking obj_tracking;
    for(int i = 0; i < cloud->points.size(); i++)
    {
    	
        //std::cout << "i = " << i << std::endl;
        if ((cl[i].x > 0) && (cl[i].x < 80))//80
        {
            
            if ((cl[i].y >= -0.50) && (cl[i].y <= 0.50))//1.5
            {
                std::cout << "F" << std::endl;
                obj_dist = sqrt(pow(cl[i].x, 2) + pow(cl[i].y, 2));
                //std::cout << "obj_dist = " << obj_dist << std::endl;
                test_cloud.push_back(cl[i]);
            }

            if ((cl[i].y > -2.5) && (cl[i].y < 2.0) && (cl[i].x > 5) && (cl[i].x < 50))
            {
                std::cout << "c[i].y = " << cl[i].y << std::endl;
                obj_steer = round(cl[i].y);
                if (obj_steer == -0)
                {
                    obj_steer = 0;
                }
                std::cout << "obj_steereeeeeeeeeeeeeeeeeeeee = " << obj_steer << std::endl;
            }
            
        }
        if ((cl[i].x > 0) && (cl[i].x < 25))//80
        {
            
            if ((cl[i].y >= -0.50) && (cl[i].y <= 0.50))//1.5
            {
                std::cout << "F" << std::endl;
                obj_hill_dist = sqrt(pow(cl[i].x, 2) + pow(cl[i].y, 2));
                //std::cout << "obj_dist = " << obj_dist << std::endl;
                test_hill_cloud.push_back(cl[i]);
            }
            
        }
       
    }
        
    std::cout << "obj_dist = " << obj_dist << std::endl;
    std::cout << "obj_hill_dist = " << obj_hill_dist << std::endl;
    std::cout << "obj_steer = " << obj_steer << std::endl;
    obj_state.header.frame_id = "obj_state";
    obj_state.header.stamp = ros::Time::now(); 
    obj_state.data = obj_dist;
    obj_hill_state.header.frame_id = "obj_hill_state";
    obj_hill_state.header.stamp = ros::Time::now();
    obj_hill_state.data = obj_hill_dist;
    obj_tracking.header.frame_id = "obj_tracking";
    obj_tracking.header.stamp = ros::Time::now(); 
    obj_tracking.data = obj_steer;
    /*std::cout << "******************************************************" << std::endl;
    std::cout << "******************************************************" << std::endl;*/
    pcl::PCLPointCloud2 center;
    pcl::toPCLPointCloud2(test_cloud, center);
    pcl::toPCLPointCloud2(test_hill_cloud, center);
    sensor_msgs::PointCloud2 output; 
    pcl_conversions::fromPCL(center, output);
    output.header.frame_id = "/velodyne"; 

    pub_dist.publish(obj_state);
    pub_hill_dist.publish(obj_hill_state);
    pub_steer.publish(obj_tracking);
    pub.publish(output); 

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object__distance");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("clustered_center_points", 1, callback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/object_judgment", 1);
    pub_steer = nh.advertise<pcl_cluster::Obj_tracking>("/Obj_tracking", 20);
    pub_dist = nh.advertise<pcl_cluster::Obj_state>("/Obj_state", 10);
    pub_hill_dist = nh.advertise<pcl_cluster::Obj_hill_state>("/Obj_hill_state", 10);
    ros::spin();
}
