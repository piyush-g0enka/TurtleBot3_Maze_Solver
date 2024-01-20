#include "parts_broadcaster.hpp"
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
#include "mage_msgs/msg/part.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

//==============================================================================
/**
 * Broadcasts parts pose and sensor pose to TF
 */
void PartsBroadcaster::static_broadcast_timer_cb_()
{
    if (parts_message != nullptr)
    {
        geometry_msgs::msg::TransformStamped static_transform_stamped;

        // Part name is extracted from Maps
        std::string part_color = parts_color_map[parts_message->part_poses[0].part.color];
        std::string part_type = parts_type_map[parts_message->part_poses[0].part.type];
        detected_part = part_color + "_" + part_type;

        static_transform_stamped.header.stamp = this->get_clock()->now();

        static_transform_stamped.header.frame_id = "logical_camera_link";
        static_transform_stamped.child_frame_id = detected_part;

        static_transform_stamped.transform.translation.x = parts_message->part_poses[0].pose.position.x;
        static_transform_stamped.transform.translation.y = parts_message->part_poses[0].pose.position.y;
        static_transform_stamped.transform.translation.z = parts_message->part_poses[0].pose.position.z;

        static_transform_stamped.transform.rotation.x = parts_message->part_poses[0].pose.orientation.x;
        static_transform_stamped.transform.rotation.y = parts_message->part_poses[0].pose.orientation.y;
        static_transform_stamped.transform.rotation.z = parts_message->part_poses[0].pose.orientation.z;
        static_transform_stamped.transform.rotation.w = parts_message->part_poses[0].pose.orientation.w;

        // Send the transform
        tf_static_broadcaster_->sendTransform(static_transform_stamped);
        // RCLCPP_INFO_STREAM(this->get_logger(), "Parts Broadcasted " << part_color + "_" + part_type);

        std_msgs::msg::String part_msg;
        part_msg.data = detected_part;
        string_publisher_->publish(part_msg);

        // Camera pose is published in TF
        geometry_msgs::msg::TransformStamped static_transform_stamped_1;

        static_transform_stamped_1.header.stamp = this->get_clock()->now();

        static_transform_stamped_1.header.frame_id = "odom";
        static_transform_stamped_1.child_frame_id = "logical_camera_link";

        static_transform_stamped_1.transform.translation.x = parts_message->sensor_pose.position.x;
        static_transform_stamped_1.transform.translation.y = parts_message->sensor_pose.position.y;
        static_transform_stamped_1.transform.translation.z = parts_message->sensor_pose.position.z;

        static_transform_stamped_1.transform.rotation.x = parts_message->sensor_pose.orientation.x;
        static_transform_stamped_1.transform.rotation.y = parts_message->sensor_pose.orientation.y;
        static_transform_stamped_1.transform.rotation.z = parts_message->sensor_pose.orientation.z;
        static_transform_stamped_1.transform.rotation.w = parts_message->sensor_pose.orientation.w;

        // Send the transform
        tf_static_broadcaster_->sendTransform(static_transform_stamped_1);
    }
}

//==============================================================================
/**
 * Callback to logical camera subscriber
 * Saves a local copy of the recent detected parts
 */
void PartsBroadcaster::parts_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{

    if (!msg->part_poses.empty())
    {

        parts_message = msg;
    }
    else
    {
        // RCLCPP_WARN(this->get_logger(), "Parts message is empty.");
        detected_part = "";
    }
}

//==============================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PartsBroadcaster>("Parts_Broadcaster");
    rclcpp::spin(node);
    rclcpp::shutdown();
}