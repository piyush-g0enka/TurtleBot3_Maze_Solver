#pragma once

#include <cmath>
#include "utils.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <map>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief RobotController class controls the Robot movement by subscribing to Aruco and Parts TF
 * @note It moves the robot by publishing to /cmd_vel
 */
class RobotController : public rclcpp::Node
{
public:
    // ==================== constructors ====================
    /**
     * @brief Construct a new Robot Controller object
     *
     * @param node_name
     */
    RobotController(std::string node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "Robot Controller ON");

        // Declare node parameters
        this->declare_parameter<std::string>("aruco_marker_0", "a");
        this->declare_parameter<std::string>("aruco_marker_1", "b");
        this->declare_parameter<std::string>("aruco_marker_2", "c");

        // Initialize state variables
        target_aruco = "";
        utils_ptr_ = std::make_shared<Utils>();
        detected_part = "";
        isStraight = true;
        isTurning = false;
        turning_direction = 1;

        // Update the parameters from params.yaml and store in a map
        aruco_map["aruco_marker_0"] = this->get_parameter("aruco_marker_0").as_string();
        aruco_map["aruco_marker_1"] = this->get_parameter("aruco_marker_1").as_string();
        aruco_map["aruco_marker_2"] = this->get_parameter("aruco_marker_2").as_string();

        // Subscribe to odom to get robot pose data
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&RobotController::odom_callback, this, _1));

        // Subscriber gets name of detected part so that the part can be looked up in TF
        parts_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/detected_parts", 10, std::bind(&RobotController::parts_callback, this, _1));

        // load a buffer of transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // timer to listen to the transforms
        listen_timer_ = this->create_wall_timer(200ms, std::bind(&RobotController::listen_aruco_transform, this));
        listen_parts_timer_ = this->create_wall_timer(500ms, std::bind(&RobotController::listen_parts_transform, this));

        // Robot velocity publisher
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Robot movement logic function
        publisher_timer_ = this->create_wall_timer(200ms, std::bind(&RobotController::move_robot, this));
    }

private:
    /*!< Boolean variable to store the value of the parameter "listen" */
    bool param_listen_;
    /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    /*!< Transform listener object */
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    /*!< Wall timer object */
    rclcpp::TimerBase::SharedPtr listen_timer_;
    rclcpp::TimerBase::SharedPtr listen_parts_timer_;

    /*!< Subscriber to odom */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    /*!< Name of aruco marker in front */
    std::string target_aruco;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr parts_subscription_;

    /*!< Robot pose */
    geometry_msgs::msg::Point aruco_position;
    geometry_msgs::msg::Point robot_position;
    geometry_msgs::msg::Point robot_orientation_euler;

    /**
     * @brief Robot state variables
     *
     */
    bool isTurning;
    bool isStraight;
    double initialYaw;
    int turning_direction;

    /*!< Publisher to publish velocity commands */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    /*!< Wall timer object */
    rclcpp::TimerBase::SharedPtr publisher_timer_;

    /*!< Utils object to access utility functions*/
    std::shared_ptr<Utils> utils_ptr_;

    std::map<std::string, std::string> aruco_map;

    std::string detected_part;
    std::map<std::string, geometry_msgs::msg::Pose> detected_parts;

    // ==================== methods ====================
    /**
     * @brief Callback function for odom subscriber
     */
    void listen_aruco_transform();

    /**
     * @brief Callback function for odom subscriber
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Move the robot based on the control decision
     */
    void move_robot();

    /**
     * @brief Compute distance between aruco marker and robot
     */
    double compute_distance(const geometry_msgs::msg::Point &robot_position, const geometry_msgs::msg::Point &aruco_position);

    /**
     * @brief Read aruco data and update control decision (turn left, right, end)
     */
    void update_aruco_target();

    /**
     * @brief Listen to detected Parts TF data and store
     */
    void listen_parts_transform();

    /**
     * @brief Callback function for parts subscriber
     */
    void parts_callback(const std_msgs::msg::String::SharedPtr msg);
};
