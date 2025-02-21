#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>

ros::Publisher marker_pub;

void read_trajectory(const std::string &filename)
{
    ROS_INFO("Trying to open file: %s", filename.c_str());

    std::ifstream file(filename);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return;
    }

    ROS_INFO("File opened successfully!");

    visualization_msgs::MarkerArray markers;
    int marker_id = 0;  // Unique ID for each marker

    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        geometry_msgs::Pose pose;
        ss >> pose.position.x >> pose.position.y >> pose.position.z;

        ROS_INFO("Read Point: x=%.3f, y=%.3f, z=%.3f", pose.position.x, pose.position.y, pose.position.z);

        // Create a marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "trajectory";
        marker.id = marker_id++;  // Assign a unique ID
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.g = 1.0; // Green color for visibility

        // ✅ Fix for Quaternion Error: Set a valid default quaternion
        marker.pose.orientation.w = 1.0;

        markers.markers.push_back(marker);
    }

    file.close();
    ROS_INFO("Total markers read: %d", marker_id);
    marker_pub.publish(markers);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_reader_publisher");
    ros::NodeHandle nh;
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/trajectory_markers", 10);

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        std::string home_dir = getenv("HOME");  
	std::string file_path = home_dir + "/trajectory.csv";
	read_trajectory(file_path);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

