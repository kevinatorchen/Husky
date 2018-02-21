#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include "extractor/feature.h"
#include "math.h"

class FeatureVisualizer {
	public:
		FeatureVisualizer();
	private:
		void extractCallback(const extractor::featureArray::ConstPtr& msg);
		ros::NodeHandle n;
		ros::Publisher feature_pub;
		ros::Subscriber sub;
};


FeatureVisualizer::FeatureVisualizer() {
	sub = n.subscribe("/feature", 1000, &FeatureVisualizer::extractCallback, this);
	feature_pub = n.advertise<visualization_msgs::Marker>("/marker", 1000);
}


void FeatureVisualizer::extractCallback(const extractor::featureArray::ConstPtr& msg)
{
	//ROS_INFO("%lu markers to publish...", features_map.size());
	for (int i = 0; i < features_map.size(); i++) {
		visualization_msgs::Marker marker;
		marker.header = msg->header;
		//marker.header.frame_id = "base_link";
    		//marker.header.stamp = ros::Time::now();
		marker.ns = "feature";
		marker.id = i;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = msg.features[i].position.x;
		marker.pose.position.y = msg.features[i].position.y;
		marker.pose.position.z = 0.0;
		marker.pose.orientation.x = 0.0;
    		marker.pose.orientation.y = 0.0;
    		marker.pose.orientation.z = 0.0;
    		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.4;
    		marker.scale.y = 0.4;
    		marker.scale.z = 1.0;
		marker.color.r = 0.0f;
    		marker.color.g = 1.0f;
    		marker.color.b = 0.0f;
    		marker.color.a = 1.0;
		marker.lifetime = ros::Duration(0.1);
		feature_pub.publish(marker);
		//ROS_INFO("I published a marker at (%f, %f)", features_map[i].getFeatureX(), features_map[i].getFeatureY());
	}

  	
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "extractor");
	FeatureVisualizer feature_extractor;
	ros::spin();
	return 0;
}
