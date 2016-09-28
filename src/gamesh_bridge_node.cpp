#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

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
#define TF_FIXED_FRAME "/world"
#define TF_MARKERSET_FRAME "/tracked_kinect"

geometry_msgs::PoseStamped globalLastPose;
bool globalLastPoseReceived;
ros::Publisher globalPointCloudPublisher;
tf::TransformListener* tfListener = NULL;
tf::TransformBroadcaster* tfBroadcaster = NULL;

void mocapPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
//	ROS_INFO("mocapPoseCallback");
	globalLastPose = *pose_msg;
	globalLastPoseReceived = true;
	
	tf::Vector3 p(pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z);
	tf::Quaternion q(pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z, pose_msg->pose.orientation.w);
//	tf::Vector3 p(1, 2, 3);
//	tf::Quaternion q(0.267261,  0.534522, 0.801784, 0.730297);

	tf::Transform transform;
	transform.setOrigin( p );
	transform.setRotation( q );
	
	tfBroadcaster->sendTransform(tf::StampedTransform(transform, pose_msg->header.stamp, TF_FIXED_FRAME, TF_MARKERSET_FRAME));
}

void pointcloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
	pcl::PointCloud<pcl::PointXYZRGB> c;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tc (new pcl::PointCloud<pcl::PointXYZRGB>());
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
//	tf::TransformListener tfListener;
	tf::StampedTransform tfTransform;
	tf::Vector3 tfOrigin;
	tf::Quaternion tfOrientation;
	geometry_msgs::PoseStamped p = globalLastPose;
//	Eigen::Vector3f origin;
//	Eigen::Quaternionf orientation;
	
	pcl::fromROSMsg(*msg, c);

	try {
		tfListener->waitForTransform(TF_FIXED_FRAME, TF_MARKERSET_FRAME, ros::Time(msg->header.stamp), ros::Duration(5.0) );
		tfListener->lookupTransform(TF_FIXED_FRAME, TF_MARKERSET_FRAME, ros::Time(msg->header.stamp), tfTransform);
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
		return;
	}
	
//	if(!globalLastPoseReceived) return; // TODO check the last pose isn't too old
	
	
//	origin = Eigen::Vector4f(p.pose.position.x, p.pose.position.y, p.pose.position.z, 0.0);
//	orientation = Eigen::Quaternionf(p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z); // := (w, x, y, z)
	
//	origin = Eigen::Vector4f(tfTransform.getOrigin().x(), tfTransform.getOrigin().y(), tfTransform.getOrigin().z(), 0.0);

	tfOrigin = tfTransform.getOrigin();
	tfOrientation = tfTransform.getRotation();
//	tfOrientation = tfTransform.getRotation().normalized();

//	origin = Eigen::Vector3f(tfOrigin.x(), tfOrigin.y(), tfOrigin.z());
//	orientation = Eigen::Quaternionf(tfOrientation.getW(), tfOrientation.getAxis().x(), tfOrientation.getAxis().y(), tfOrientation.getAxis().z()); // := (w, x, y, z)
	
//	c.sensor_origin_ = origin;
//	c.sensor_orientation_ = orientation;
	
//	ROS_INFO("origin:\t\t %f \t %f \t %f", origin[0], origin[1], origin[2]);
//	ROS_INFO("orientation:\t %f \t %f \t %f \t %f", orientation.x(), orientation.y(), orientation.z(), orientation.w());
	
	
	transform.rotate(Eigen::Quaternionf(tfOrientation.getW(), tfOrientation.getAxis().x(), tfOrientation.getAxis().y(), tfOrientation.getAxis().z()));
	transform.translate(Eigen::Vector3f(tfOrigin.x(), tfOrigin.y(), tfOrigin.z()));
//	transform.translation() << c.sensor_origin_[0], c.sensor_origin_[1], c.sensor_origin_[2];
	
//	std::cout << "transform matrix:" << std::endl << transform.matrix() << std::endl << std::endl;
	
//	std::cout << "tf transform pose:\tp.x: " << tfOrigin.x() << ",\tp.y:    " << tfOrigin.y() << ",\tp.z: " << tfOrigin.z() << std::endl << ",\to.x: " << tfOrientation.getAxis().x() << ",\to.y: " << tfOrientation.getAxis().y() << ",\to.z: " << tfOrientation.getAxis().z() << ",\to.w: " << tfOrientation.getW() << std::endl;
//	std::cout << "cb transform pose:\tp.x: " << p.pose.position.x << ",\tp.y:    " << p.pose.position.y << ",\tp.z: " << p.pose.position.z << std::endl << ",\to.x: " << p.pose.orientation.x << ",\to.y: " << p.pose.orientation.y << ",\to.z: " << p.pose.orientation.z << ",\to.w: " << p.pose.orientation.w << std::endl;
	std::cout << "tf transform o diff:\t" << "\to.x: " << tfOrientation.getAxis().x()-p.pose.orientation.x << ",   \to.y: " << tfOrientation.getAxis().y()-p.pose.orientation.y << ",   \to.z: " << tfOrientation.getAxis().z()-p.pose.orientation.z << ",   \to.w: " << tfOrientation.getW()-p.pose.orientation.w << std::endl;
	std::cout << "tf transform o tf  :\t" << "\to.x: " << tfOrientation.getAxis().x() << ",   \to.y: " << tfOrientation.getAxis().y() << ",   \to.z: " << tfOrientation.getAxis().z() << ",   \to.w: " << tfOrientation.getW() << std::endl;
	std::cout << "cb transform o cb  :\t" << "\to.x: " << p.pose.orientation.x << ",   \to.y: " << p.pose.orientation.y << ",   \to.z: " << p.pose.orientation.z << ",   \to.w: " << p.pose.orientation.w << std::endl << std::endl ;
	
//	std::cout << "time diffs:\t" << p.header.stamp << "\t,\t" << c.header.stamp << std::endl;
	
	transform.rotate(Eigen::AngleAxis<float>(-1.5707963267948966, Eigen::Vector3f(1,0,0)));
	transform.rotate(Eigen::AngleAxis<float>( 1.5707963267948966, Eigen::Vector3f(0,1,0)));
	
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
	ros::init(argc, argv, "bridge");
	ROS_INFO("Node initialised");
	
	ros::NodeHandle n;
	
	ros::Subscriber mocapPoseSubscriber = n.subscribe<geometry_msgs::PoseStamped> (MOCAP_POSE_TOPIC, 1000, mocapPoseCallback);
	ros::Subscriber pc2Subscriber = n.subscribe<sensor_msgs::PointCloud2> (POINTCLOUD2_TOPIC, 1000, pointcloud2Callback);
	
	globalPointCloudPublisher = n.advertise<pcl::PCLPointCloud2>("translated_points", 1);
	
	tfListener = (new tf::TransformListener);
	tfBroadcaster = (new tf::TransformBroadcaster);

	ros::spin();
	
	return 0;
}

