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

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief Class ArucoBroadcaster: Subscribes to Aruck Marker messages and publishes them to TF
 *
 */
class ArucoBroadcaster : public rclcpp::Node
{
public:
    // ==================== constructors ====================
    /**
     * @brief Construct a new Aruco Broadcaster object
     *
     * @param node_name
     */
    ArucoBroadcaster(std::string node_name) : Node(node_name)
    {

        RCLCPP_INFO(this->get_logger(), "Aruco TF Broadcaster started...");

        aruco_message = nullptr;

        // Subscriber for Aruco Marker messages
        aruco_subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "/aruco_markers", 10, std::bind(&ArucoBroadcaster::aruco_callback, this, _1));

        // initialize a static transform broadcaster
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Load a buffer of transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_buffer_->setUsingDedicatedThread(true);

        // Create a utils object to use the utility functions
        utils_ptr_ = std::make_shared<Utils>();

        // timer to publish the transform
        static_broadcast_timer_ = this->create_wall_timer(
            100ms, std::bind(&ArucoBroadcaster::static_broadcast_timer_cb_, this));
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

    /*!< Aruco subscriber*/
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscription_;

    /*!< Store latest Aruco message*/
    ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr aruco_message;

    // ==================== methods ====================
    /**
     * @brief Timer to broadcast the transform of the Aruco Marker
     *
     */
    void static_broadcast_timer_cb_();

    /**
     * @brief Callback functin for Aruco Subscriber
     *
     */
    void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);
};
