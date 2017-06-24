#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc,char** argv)
{
	ros::init(argc,argv,"node_sensors_tf");
	ros::NodeHandle nodeHandle;
	tf::TransformBroadcaster tfBroadcaster;
	tf::Transform baselink2LaserTf = tf::Transform(tf::Quaternion(0,0,0),tf::Vector3(0.33,0,0));
	tf::Transform baselink2CompassTf = tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,0));
	ros::Rate loop_rate(20);
	while(nodeHandle.ok())
	{
		tfBroadcaster.sendTransform(
			tf::StampedTransform(baselink2LaserTf,ros::Time::now(),"base_link","laser"));
		tfBroadcaster.sendTransform(
			tf::StampedTransform(baselink2CompassTf,ros::Time::now(),"base_link","compass"));
		loop_rate.sleep();
	}	
}
