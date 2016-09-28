#include "ros/ros.h"

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
//#include <tf/LinearMath/Transform.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

//using namespace std;

#define POINTCLOUD2_TOPIC "/kinect2/sd/points"
#define MOCAP_POSE_TOPIC "/tracked_kinect/pose"

geometry_msgs::PoseStamped globalLastPose;
bool globalLastPoseReceived;
ros::Publisher globalPointCloudPublisher;


void mocapPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
//	ROS_INFO("mocapPoseCallback");
	globalLastPose = *pose_msg;
	globalLastPoseReceived = true;
}

void pointcloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
	pcl::PointCloud<pcl::PointXYZRGB> c;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tc (new pcl::PointCloud<pcl::PointXYZRGB>());
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	geometry_msgs::PoseStamped p = globalLastPose;
	
	if(!globalLastPoseReceived) return; // TODO check the last pose isn't too old
	
	pcl::fromROSMsg(*msg, c);
	
	Eigen::Vector4f origin(p.pose.position.x, p.pose.position.y, p.pose.position.z, 0.0);
	Eigen::Quaternionf orientation(p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z); // := (w, x, y, z)
	
	c.sensor_origin_ = origin;
	c.sensor_orientation_ = orientation;
	
//	ROS_INFO("origin:\t\t %f \t %f \t %f", c.sensor_origin_[0], c.sensor_origin_[1], c.sensor_origin_[2]);
//	ROS_INFO("orientation:\t %f \t %f \t %f \t %f", -c.sensor_orientation_.x(), c.sensor_orientation_.y(), c.sensor_orientation_.z(), c.sensor_orientation_.w());
	
	
	transform.rotate(orientation);
	transform.translation() << c.sensor_origin_[0], c.sensor_origin_[1], c.sensor_origin_[2];
	
//	std::cout << "transform matrix:" << std::endl << transform.matrix() << std::endl;
	
	std::cout << "time diffs:\t" << p.header.stamp - msg->header.stamp << std::endl;
	
	transform.rotate(Eigen::AngleAxis<float>(-1.5707963267948966, Eigen::Vector3f(1,0,0)));
	transform.rotate(Eigen::AngleAxis<float>(1.5707963267948966, Eigen::Vector3f(0,1,0)));

	// Executing the transformation
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tc (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::transformPointCloud(c, *tc, transform);
	
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr publishedPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//	publishedPointCloud->header.frame_id = "world";
//	publishedPointCloud->height = tc->height;
//	publishedPointCloud->width = tc->width;
//	publishedPointCloud->points.push_back(tc->points.begin(), tc->points.end());

//	publishedPointCloud->header.stamp = ros::Time::now().toNSec();


	pcl::PCLPointCloud2::Ptr ptc (new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2(*tc, *ptc);
	globalPointCloudPublisher.publish(ptc);

}



int main(int argc, char **argv) {
	ros::init(argc, argv, "listener");
	ROS_INFO("Node initialised");
	
	ros::NodeHandle n;
	
	ros::Subscriber mocapPoseSubscriber = n.subscribe<geometry_msgs::PoseStamped> (MOCAP_POSE_TOPIC, 1000, mocapPoseCallback);
	ros::Subscriber pc2Subscriber = n.subscribe<sensor_msgs::PointCloud2> (POINTCLOUD2_TOPIC, 1000, pointcloud2Callback);

	globalPointCloudPublisher = n.advertise<pcl::PCLPointCloud2>("translated_points", 1);

	ros::spin();
	
	return 0;
}

