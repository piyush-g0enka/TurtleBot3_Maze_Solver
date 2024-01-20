#include "aruco_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "utils.hpp"
#include <tf2/exceptions.h>
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <iostream>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

//==============================================================================
// This function broadcasts the aruco marker pose in TF static topic
void ArucoBroadcaster::static_broadcast_timer_cb_()
{
    if (aruco_message != nullptr)
    {
        // We get data of 3 aruco markers out of which one marker is desired
        size_t arrayLength = sizeof(aruco_message->marker_ids) / sizeof(aruco_message->marker_ids[0]);
        size_t minIndex = 0;
        double mZ = 9999.9; // Initialize to a large value

        // Traverse the array using a for loop
        for (size_t i = 0; i < (arrayLength - 1); ++i)
        {
            // Check if the current z value is smaller than the minimum and if it is greater than 0.5 [min range of camera]
            if ((std::round(aruco_message->poses[i].position.z * 1000) / 1000.0 <= mZ) && (std::round(aruco_message->poses[i].position.z * 1000) / 1000.0 >= 0.5))
            {
                mZ = aruco_message->poses[i].position.z;
                minIndex = i; // Update the index of the minimum z value
            }
        }

        // RCLCPP_INFO_STREAM(this->get_logger(), "ARUCO id "<<minIndex);

        // Once the desired frame is selected, we publish it in TF
        geometry_msgs::msg::TransformStamped static_transform_stamped;

        static_transform_stamped.header.stamp = this->get_clock()->now();
        static_transform_stamped.header.frame_id = "camera_rgb_optical_frame";
        static_transform_stamped.child_frame_id = "aruco";

        static_transform_stamped.transform.translation.x = aruco_message->poses[minIndex].position.x;
        static_transform_stamped.transform.translation.z = aruco_message->poses[minIndex].position.z;

        static_transform_stamped.transform.rotation.w = aruco_message->poses[minIndex].orientation.w;
        static_transform_stamped.transform.rotation.y = aruco_message->poses[minIndex].orientation.y;
        static_transform_stamped.transform.rotation.z = aruco_message->poses[minIndex].orientation.z;
        static_transform_stamped.transform.rotation.x = aruco_message->poses[minIndex].orientation.x;

        // We manipulate value of y to contain information of aruco id
        static_transform_stamped.transform.translation.y = aruco_message->marker_ids[minIndex];
        // Send the transform
        tf_static_broadcaster_->sendTransform(static_transform_stamped);
        // RCLCPP_INFO_STREAM(this->get_logger(), "ARUCO Broadcasted " << static_transform_stamped.transform.translation.y);
    }
}

//==============================================================================
/*
 * This function is called by our aruco subscriber whenever aruco is detected.
 * Here we save a copy of the message received so that the above function can process it and publish in TF
 */
void ArucoBroadcaster::aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
    // check if marker id is empty or not
    if (!msg->marker_ids.empty())
    {
        // RCLCPP_INFO(this->get_logger(), "ARUCO Subscribed");
        aruco_message = msg; // Save message data in member attribute
    }
    else
    {
        // RCLCPP_WARN(this->get_logger(), "ArucoMarkers message is empty.");
        ;
    }
}

//==============================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoBroadcaster>("Aruco_Broadcaster");
    rclcpp::spin(node);
    rclcpp::shutdown();
}