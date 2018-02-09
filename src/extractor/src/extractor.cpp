#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include "math.h"

const float MAX_DIST = 40.0;
const float DIFF_THRES = 15.0;
const float MAX_DIAMETER = 1.5;

//Uses law of cosines.
float computeDiameter(float side1, float side2, float angle) {
	return sqrt(pow(side1, 2.0) + pow(side2, 2.0) - 2.0 * side1 * side2 * cos(angle));
}

class Feature {
	private:
		float theta1;
		float range1;
		float theta2;
		float range2;
		float diameter;
		float featureX;
		float featureY;
	public:
		int a;
		int b;
		Feature(float, float, float, float);
		float getFeatureX();
		float getFeatureY();
};

Feature::Feature(float theta1, float range1, float theta2, float range2) : theta1(theta1), range1(range1), theta2(theta2), range2(range2) {
	diameter = computeDiameter(range1, range2, theta2 - theta1);

	float x1 = range1 * cos(theta1);
	float y1 = range1 * sin(theta1);
	float x2 = range2 * cos(theta2);
	float y2 = range2 * sin(theta2);
	featureX = (x1 + x2) / 2.0;
	featureY = (y1 + y2) / 2.0;
}

float Feature::getFeatureX() {
	return featureX;
}

float Feature::getFeatureY() {
	return featureY;
}

class FeatureExtractor {
	public:
		FeatureExtractor();
	private:
		void extractCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
		ros::NodeHandle n;
		ros::Publisher feature_pub;
		ros::Subscriber sub;
};


FeatureExtractor::FeatureExtractor() {
	sub = n.subscribe("/lidar/scan", 1000, &FeatureExtractor::extractCallback, this);
	feature_pub = n.advertise<visualization_msgs::Marker>("/feature", 1000);
}


void FeatureExtractor::extractCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	std::vector<float> ranges = msg->ranges;
	for (int i = 0; i < ranges.size(); i++) {
		if (ranges[i] == 0 || ranges[i] > MAX_DIST) {
			ranges[i] = MAX_DIST;
		}
	}
	std::vector<Feature> features_map;
	int featureIndex = -1;
	bool lookForNegativePeak = true;
	for (int i = 0; i < ranges.size() - 1; i++) {
		float diff = ranges[i + 1] - ranges[i];
		if (lookForNegativePeak) {
			if (diff < -1 * DIFF_THRES) {
				featureIndex = i + 1;
				lookForNegativePeak = false;
			}
		} else {
			float angle = msg->angle_increment * (featureIndex - i);
			float diameter = computeDiameter(ranges[featureIndex], i, angle);
			if (diff > DIFF_THRES) {
				float theta1 = msg->angle_min + (msg->angle_increment * featureIndex);
				float theta2 = msg->angle_min + (msg->angle_increment * i);
				Feature feature(theta1, ranges[featureIndex], theta2, ranges[i]);
				ROS_INFO("Created feature with Theta1: %f, Range1: %f, Theta2: %f, Range2: %f", 
					theta1, ranges[featureIndex], theta2, ranges[i]);
				features_map.push_back(feature);
				featureIndex = -1;
				lookForNegativePeak = true;				
			} else if (diameter > MAX_DIAMETER) {
				featureIndex = -1;
				lookForNegativePeak = true;
			}
		}
		
	}
	//ROS_INFO("%lu markers to publish...", features_map.size());
	for (int i = 0; i < features_map.size(); i++) {
		visualization_msgs::Marker marker;
		//marker.header.frame_id = msg->header.frame_id;
		marker.header.frame_id = "base_link";
    		marker.header.stamp = ros::Time::now();
		marker.ns = "feature";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = features_map[i].getFeatureX();
		marker.pose.position.y = features_map[i].getFeatureY();
		marker.pose.position.z = 0.0;
		marker.pose.orientation.x = 0.0;
    		marker.pose.orientation.y = 0.0;
    		marker.pose.orientation.z = 0.0;
    		marker.pose.orientation.w = 1.0;
		marker.scale.x = 1.0;
    		marker.scale.y = 1.0;
    		marker.scale.z = 1.0;
		marker.color.r = 0.0f;
    		marker.color.g = 1.0f;
    		marker.color.b = 0.0f;
    		marker.color.a = 1.0;
		marker.lifetime = ros::Duration();
		feature_pub.publish(marker);
		//ROS_INFO("I published a marker at (%f, %f)", features_map[i].getFeatureX(), features_map[i].getFeatureY());
	}

  	
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "extractor");
	FeatureExtractor feature_extractor;
	ros::spin();
	return 0;
}
