#pragma once

#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include "utils.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief PartsBroadcaster broadcasts detected parts pose in TF
 * @note It also send a message to our controller notifying of the parts detected
 */
class PartsBroadcaster : public rclcpp::Node
{
public:
    // ==================== constructors ====================
    /**
     * @brief Construct a new Parts Broadcaster object
     *
     * @param node_name
     */
    PartsBroadcaster(std::string node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "Parts TF Broadcaster started");

        // Created Maps to extract parts color and type from int data
        parts_color_map[0] = "Red";
        parts_color_map[1] = "Green";
        parts_color_map[2] = "Blue";
        parts_color_map[3] = "Orange";
        parts_color_map[4] = "Purple";

        parts_type_map[10] = "Battery";
        parts_type_map[11] = "Pump";
        parts_type_map[12] = "Sensor";
        parts_type_map[13] = "Regulator";

        detected_part = "";

        parts_message = nullptr;

        // Create a compatible Quality of Service with the sensor
        rclcpp::QoS qos = rclcpp::SensorDataQoS();

        // Subscribe to Logical camera
        parts_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/advanced_logical_camera/image", qos, std::bind(&PartsBroadcaster::parts_callback, this, _1));

        // Initialize the static transform broadcaster
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Load a buffer of transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_buffer_->setUsingDedicatedThread(true);

        // Create a utils object to use the utility functions
        utils_ptr_ = std::make_shared<Utils>();

        // timer to publish the transform
        static_broadcast_timer_ = this->create_wall_timer(
            500ms, std::bind(&PartsBroadcaster::static_broadcast_timer_cb_, this));

        // Publisher to publish detected part name to controller
        string_publisher_ = create_publisher<std_msgs::msg::String>("detected_parts", 10);
    }

private:
    /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    /*!< Static broadcaster object */
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    /*!< Utils object to access utility functions*/
    std::shared_ptr<Utils> utils_ptr_;

    /*!< Wall timer object for the static broadcaster*/
    rclcpp::TimerBase::SharedPtr static_broadcast_timer_;

    /*!< Subscriber for Logical Camera*/
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr parts_subscription_;

    /*!< Pointer to store parts message which would be used by TF broadcaster*/
    mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr parts_message;

    /*!< Maps declaration*/
    std::map<int, std::string> parts_color_map;
    std::map<int, std::string> parts_type_map;

    std::string detected_part;
    std::map<std::string, geometry_msgs::msg::Pose> detected_parts;

    /*!< Publisher to send detected part names to controller*/
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher_;

    /**
     * @brief Broadcast the transform of the detected part
     *
     * @private
     */
    void static_broadcast_timer_cb_();

    /**
     * @brief Callback function for logical camera subscriber
     *
     * @param msg
     */
    void parts_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
};
