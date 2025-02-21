#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>
#include <fstream>
#include <vector>
#include <string>
#include "trajectory_manager/SaveTrajectory.h"

struct TrajectoryPoint {
    geometry_msgs::Point position;
    std_msgs::Header header;
};

std::vector<TrajectoryPoint> trajectory;
ros::Publisher marker_pub;

// Callback for Odometry data
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    TrajectoryPoint point;
    point.position.x = msg->pose.pose.position.x;
    point.position.y = msg->pose.pose.position.y;
    point.position.z = msg->pose.pose.position.z;
    point.header.stamp = ros::Time::now();
    
    trajectory.push_back(point);
    ROS_DEBUG("Recorded trajectory point: x=%.3f, y=%.3f, z=%.3f", 
               point.position.x, point.position.y, point.position.z);
}

// Function to publish markers in RViz
void publishMarkers() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectory";
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0; // Red color

    for (const auto& t : trajectory) {
        marker.points.push_back(t.position);
    }

    marker_pub.publish(marker);
}

// Service to save trajectory
bool saveTrajectory(trajectory_manager::SaveTrajectory::Request &req,
                    trajectory_manager::SaveTrajectory::Response &res) {
    ROS_INFO("Service called: saveTrajectory");
    ROS_INFO("Requested filename: %s", req.filename.c_str());
    ROS_INFO("Requested duration: %.2f seconds", req.duration);

    // Ensure trajectory is not empty before saving
    if (trajectory.empty()) {
        ROS_WARN("Trajectory is empty. No data to save.");
        res.success = false;
        res.message = "Trajectory is empty, nothing to save.";
        return true;
    }

    // Attempt to open the file for writing
    std::ofstream file(req.filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", req.filename.c_str());
        res.success = false;
        res.message = "Failed to open file.";
        return true;
    }

    ROS_INFO("Writing trajectory data to file...");
    int count = 0;

    // Get the current time for filtering by duration
    ros::Time current_time = ros::Time::now();
    ros::Duration duration_limit(req.duration);

    for (auto it = trajectory.rbegin(); it != trajectory.rend(); ++it) {
        ros::Duration time_diff = current_time - it->header.stamp;
        if (time_diff > duration_limit) {
            break; // Stop when we exceed the required duration
        }

        file << it->position.x << "," << it->position.y << "," << it->position.z << "\n";
        count++;
    }

    file.close();
    ROS_INFO("Trajectory saved successfully to: %s", req.filename.c_str());
    ROS_INFO("Total points saved: %d", count);

    res.success = true;
    res.message = "Trajectory saved successfully.";
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_saver");
    ros::NodeHandle nh;

    // Subscriber to /odom to collect trajectory data
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1000, odomCallback);

    // Publisher to visualize trajectory in RViz
    marker_pub = nh.advertise<visualization_msgs::Marker>("trajectory_markers", 10);

    // Service to save trajectory to file
    ros::ServiceServer service = nh.advertiseService("save_trajectory", saveTrajectory);

    ROS_INFO("Trajectory Saver Node is running...");

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        publishMarkers();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
