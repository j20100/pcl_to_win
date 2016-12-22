#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/filters/filter.h>
#include <pcl_to_windows/PCLXYZRGB.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_accumulated(new pcl::PointCloud<pcl::PointXYZRGB>);

void
cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  ros::Rate loop_rate(15);
  // Container for original & filtered data
  pcl::PCLPointCloud2::Ptr pcl2 (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr pcl2_filtered (new pcl::PCLPointCloud2 ());

  // Ros msg
  pcl_to_windows::PCLXYZRGB msg;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *pcl2);

  // Decimated point cloud
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (pcl2);
  sor.setLeafSize (0.02f, 0.02f, 0.02f);
  sor.filter (*pcl2_filtered);
  //std::cerr << "PointCloud before filtering: " << pcl2->width * pcl2->height 
  //     << " data points (" << pcl::getFieldsList (*pcl2) << ").";
  //std::cerr << "PointCloud after filtering: " << pcl2_filtered->width * pcl2_filtered->height 
  //     << " data points (" << pcl::getFieldsList (*pcl2_filtered) << ").";

  // Cloud transformation to XYZRGB
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(*pcl2_filtered, *cloud);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud_out, indices);

  // Reserve memory for ros msg
  msg.x.reserve(cloud->points.size());
  msg.y.reserve(cloud->points.size());
  msg.z.reserve(cloud->points.size());
  msg.r.reserve(cloud->points.size());
  msg.g.reserve(cloud->points.size());
  msg.b.reserve(cloud->points.size());

  int cld_length = 0;
  int cld_num = 0;
  int cld_total = cloud_out->points.size();
  int batch_size = 2000;
  cld_length = cld_total/batch_size;
  ROS_INFO( "PointCloud total: %i", cld_total);
  ROS_INFO( "batch_size: %i", batch_size);

  // Cloud publication
  for (size_t k = 0; k < cloud_out->points.size (); k=k+batch_size){
    for (size_t i = 0; i < batch_size; ++i){
      msg.x.push_back(cloud_out->points[k+i].x);
      msg.y.push_back(cloud_out->points[k+i].y);
      msg.z.push_back(cloud_out->points[k+i].z);
      msg.r.push_back(cloud_out->points[k+i].r);
      msg.g.push_back(cloud_out->points[k+i].g);
      msg.b.push_back(cloud_out->points[k+i].b);
    };

    msg.cld_total = cld_total;
    msg.cld_num = k;
    //ROS_INFO("K: %i", k);
    //std::cerr << "PointCloud length aafter: " << cld_length << "/n";
    //std::cerr << "PointCloud number: " << cld_num << "/n";
    pub.publish(msg);

    msg.x.clear();
    msg.y.clear();
    msg.z.clear();
    msg.r.clear();
    msg.g.clear();
    msg.b.clear();

    cld_num = cld_num+1;
    loop_rate.sleep();
  };
  ROS_INFO("PCL sent");
}

int
main (int argc, char** argv)
{
  ROS_INFO("INIT");
  ros::init (argc, argv, "pcl_to_windows");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_callback);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl_to_windows::PCLXYZRGB> ("pcl_xyzrgb", 1);


  while (ros::ok())
  {
    // Spin
    ros::spinOnce();
    //loop_rate.sleep();
  };

}
