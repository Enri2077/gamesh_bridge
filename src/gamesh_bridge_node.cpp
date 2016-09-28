#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <gamesh_bridge/GameshRays.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
//#include <tf/LinearMath/Transform.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

//using namespace std;

#define POINTCLOUD2_TOPIC "/kinect2/sd/points"
#define TF_FIXED_FRAME "/world"
#define TF_MARKERSET_FRAME "/tracked_kinect"

long int globalCameraId = 0;

ros::Publisher globalPointCloudPublisher;
ros::Publisher globalGameshRaysPublisher;
tf::TransformListener* tfListener = NULL;
tf::TransformBroadcaster* tfBroadcaster = NULL;


void pointcloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
	pcl::PointCloud<pcl::PointXYZRGB> c;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tc (new pcl::PointCloud<pcl::PointXYZRGB>());
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	tf::StampedTransform tfTransform;
	tf::Vector3 tfOrigin;
	tf::Quaternion tfOrientation;
	
	gamesh_bridge::GameshRays grm;
	
	pcl::fromROSMsg(*msg, c);

	try {
		tfListener->waitForTransform(TF_FIXED_FRAME, TF_MARKERSET_FRAME, ros::Time(msg->header.stamp), ros::Duration(1.0) );
		tfListener->lookupTransform(TF_FIXED_FRAME, TF_MARKERSET_FRAME, ros::Time(msg->header.stamp), tfTransform);
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
		return;
	}
	
	tfOrigin = tfTransform.getOrigin();
	tfOrientation = tfTransform.getRotation();
	tf::Matrix3x3 tf_rotation_matrix(tfOrientation);
	double roll, pitch, yaw;
	tf_rotation_matrix.getRPY(roll, pitch, yaw);
	
//	std::cout << "tf p   :\t" << "\tx: " << tfOrigin.x() << ",   \ty: " << tfOrigin.y() << ",   \tz: " << tfOrigin.z() << std::endl;
//	std::cout << "tf o   :\t" << "\tr: " << roll << ",   \tp: " << pitch << ",   \ty: " << yaw << std::endl;
	
	// world coordinates markerset's transformation
	transform.translate(Eigen::Vector3f(tfOrigin.x(), tfOrigin.y(), tfOrigin.z()));
	transform.rotate(Eigen::AngleAxis<float>(yaw, Eigen::Vector3f(0,0,1)));
	transform.rotate(Eigen::AngleAxis<float>(roll, Eigen::Vector3f(1,0,0)));
	transform.rotate(Eigen::AngleAxis<float>(pitch, Eigen::Vector3f(0,1,0)));
	
	// Kinect's frame transformation
	transform.rotate(Eigen::AngleAxis<float>(-1.5707963267948966, Eigen::Vector3f(1,0,0)));
	transform.rotate(Eigen::AngleAxis<float>( 1.5707963267948966, Eigen::Vector3f(0,1,0)));
	
	// Executing the transformation
	pcl::transformPointCloud(c, *tc, transform);
	
	// Publish the translated pointcloud
	pcl::PCLPointCloud2::Ptr pc2m (new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2(*tc, *pc2m);
	globalPointCloudPublisher.publish(pc2m);
	
	// Publish the gamesh rays
	sensor_msgs::PointCloud2::Ptr ros_pc2m (new sensor_msgs::PointCloud2());
	pcl::toROSMsg(*tc, *ros_pc2m);
	grm.pointcloud = *ros_pc2m;

	grm.camera_id = globalCameraId++;

	grm.camera_pose.position.x = tfOrigin.x();
	grm.camera_pose.position.y = tfOrigin.y();
	grm.camera_pose.position.z = tfOrigin.z();
	grm.camera_pose.orientation.x = tfOrientation.x();
	grm.camera_pose.orientation.y = tfOrientation.y();
	grm.camera_pose.orientation.z = tfOrientation.z();
	grm.camera_pose.orientation.w = tfOrientation.w();
	
	globalGameshRaysPublisher.publish(grm);
	
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "bridge");
	ROS_INFO("Gamesh bridge initialised");
	
	ros::NodeHandle n;
	
	ros::Subscriber pc2Subscriber = n.subscribe<sensor_msgs::PointCloud2> (POINTCLOUD2_TOPIC, 1000, pointcloud2Callback);
	
	globalPointCloudPublisher = n.advertise<pcl::PCLPointCloud2>("translated_points", 1);
	globalGameshRaysPublisher = n.advertise<gamesh_bridge::GameshRays>("gamesh_rays", 1);
	
	tfListener = (new tf::TransformListener);
	tfBroadcaster = (new tf::TransformBroadcaster);

	ros::spin();
	
	return 0;
}

