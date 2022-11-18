#include <ros/ros.h>
#include <iterator>
#include <typeinfo>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

// Euclidean Cluster Extraction
// http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction

ros::Publisher pub;
ros::Publisher pub2;
using std::copy; using std::ostream_iterator;
void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  //std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl;

  // 탐색을 위한 KdTree 오브젝트 생성 //Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);  //KdTree 생성 


  std::vector<pcl::PointIndices> cluster_indices;       // 군집화된 결과물의 Index 저장, 다중 군집화 객체는 cluster_indices[0] 순으로 저장 
  // 군집화 오브젝트 생성  
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setInputCloud (cloud);       // 입력   
  ec.setClusterTolerance (0.7);  // 2cm  0.02 0.15
  ec.setMinClusterSize (6);     // 최소 포인트 수 100
  ec.setMaxClusterSize (200);   // 최대 포인트 수 25000
  ec.setSearchMethod (tree);      // 위에서 정의한 탐색 방법 지정 
  ec.extract (cluster_indices);   // 군집화 적용 
  
  std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;///1
  //copy(cluster_indices.begin(), cluster_indices.end(), ostream_iterator<pcl::PointIndices>(std::cout, "; "));
  //std::cout << std::endl;
  //std::cout << "cluster_indices.size 11111 ====>   " << cluster_indices.size() << std::endl;
  //std::cout << "cluster_indices" << cluster_indices.data () << std::endl;//0x559d14587c60
  //std::cout << typeid(cluster_indices.data ()).name() << std::endl;//PN3pcl12PointIndicesE
  pcl::PointCloud<pcl::PointXYZI> TotalCloud;
  
  // 클러스터별 정보 수집, 출력, 저장 
  int j = 0;
  auto x_sum = 0;
  auto y_sum = 0;
  auto z_sum = 0;
  auto flag = 0;
  pcl::PointCloud<pcl::PointXYZ> cloud_center;
  pcl::PointXYZ c1;
  //cluster
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    
    //pcl::PointXYZ c1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    //points in each cluster
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {

      cloud_cluster->points.push_back (cloud->points[*pit]);

      pcl::PointXYZ pt = cloud->points[*pit];
      pcl::PointXYZI pt2;
      pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;

      if (abs(pt.y) > 2.0)
      {
        flag = 1;
        //std::cout << "here pt.y < 2.0   " << std::endl;
        
      }
      
      pt2.intensity = (float)(j + 1);
      TotalCloud.push_back(pt2);
      
    }
    
    std::cout << "cloud_cluster->points.size() = out = " << cloud_cluster->points.size() << std::endl;
    
    /*
    if (cloud_cluster->points.size()< 20) 
    {
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
    }*/
    
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    pcl::CentroidPoint<pcl::PointXYZ> centroid;
    
    for(int i = 0; i < cloud_cluster->points.size(); i++)
    {
      
      if((cloud_cluster->points.size() < 200) && (cloud_cluster->points.size() > 3))
      {
        std::cout << "cloud_cluster->points.size() = " << cloud_cluster->points.size() << std::endl;
        centroid.add(pcl::PointXYZ ((*cloud_cluster)[i].x, (*cloud_cluster)[i].y, (*cloud_cluster)[i].z));
      }

    }
    if (flag == 0)
    {
      //std::cout << "flag222 = " << flag << std::endl;
      centroid.get(c1);  
      cloud_center.push_back(c1);
      
    }
    flag = 0;
    j++;

  }

  
  pcl::PCLPointCloud2 cloud_p;
  pcl::PCLPointCloud2 center;
  pcl::toPCLPointCloud2(cloud_center, center);
  pcl::toPCLPointCloud2(TotalCloud, cloud_p);//transform to PointCloud2 of sensor_msgs 
  //std::cout << "TotalCloud = " << TotalCloud.data () << std::endl;
  sensor_msgs::PointCloud2 output; 
  sensor_msgs::PointCloud2 output_center;
  pcl_conversions::fromPCL(center, output_center);
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = "/velodyne";  
  output_center.header.frame_id = "/velodyne"; 
  pub.publish(output_center);
  pub2.publish(output);
  //std::cout << output << std::endl; 
 
  //ROS_INFO("published it.");
}


int 
main (int argc, char** argv)
{
  ros::init(argc, argv, "cluster");
  ros::NodeHandle nh;
  ros::NodeHandle nh2;
  ros::Subscriber sub = nh.subscribe("/delete_ground_combined", 1, callback);
  ////ros::Subscriber sub = nh.subscribe("/Combined_velo", 1, callback);
  pub = nh.advertise<sensor_msgs::PointCloud2>("/clustered_center_points",1);
  pub2 = nh2.advertise<sensor_msgs::PointCloud2>("/clustered_points",1);
  
  ros::spin();
}

