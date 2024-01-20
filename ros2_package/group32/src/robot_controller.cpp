#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "robot_controller.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "utils.hpp"
#include <map>
#include <cmath>
#include <tf2/exceptions.h>

using namespace std::chrono_literals;

//==============================================================================
/**
 * Listner of Aruco TF
 * Gets aruco position and aruco_id for control logic
 */
void RobotController::listen_aruco_transform()
{
    std::string target_frame = "aruco";
    std::string source_frame = "odom";
    geometry_msgs::msg::TransformStamped t_stamped;
    geometry_msgs::msg::Pose pose_out;
    try
    {
        t_stamped = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 50ms);
    }
    catch (const tf2::TransformException &ex)
    {
        // RCLCPP_WARN_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return;
    }

    pose_out.position.x = t_stamped.transform.translation.x;
    pose_out.position.y = t_stamped.transform.translation.y;
    pose_out.position.z = t_stamped.transform.translation.z;
    pose_out.orientation = t_stamped.transform.rotation;

    // We encrypt aruco_id data in z-value since z-value is not used in calculation of distance between robot and aruco

    if (t_stamped.transform.translation.z > 0.0)
        target_aruco = "aruco_marker_0";
    else if (t_stamped.transform.translation.z > -1.0)
        target_aruco = "aruco_marker_1";
    else if (t_stamped.transform.translation.z > -2.0)
        target_aruco = "aruco_marker_2";

    // Save the aruco position for processing by other function
    aruco_position.x = pose_out.position.x;
    aruco_position.y = pose_out.position.y;
    aruco_position.z = pose_out.position.z;
}

//==============================================================================
/**
 * Get Robot pose data
 */
void RobotController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{

    robot_position.x = msg->pose.pose.position.x;
    robot_position.y = msg->pose.pose.position.y;
    robot_position.z = msg->pose.pose.position.z;

    tf2::Quaternion quat_tf;
    tf2::convert(msg->pose.pose.orientation, quat_tf);

    std::array<double, 3> rpy;

    rpy = utils_ptr_->set_euler_from_quaternion(quat_tf);
    robot_orientation_euler.x = rpy.at(0);
    robot_orientation_euler.y = rpy.at(1);
    robot_orientation_euler.z = rpy.at(2);
}

//==============================================================================
/**
 * Logic to move robot
 */
void RobotController::move_robot()
{
    double d_robot_aruco{};
    geometry_msgs::msg::Twist cmd_vel;

    // This condition is true only when robot reaches end
    if (turning_direction == 0)
    {
        isStraight = false;
        isTurning = false;

        std::cout << "=========================================================================================\n";
        std::cout << "-----------------------------------------------------------------------------------------\n";
        for (auto it = detected_parts.begin(); it != detected_parts.end(); ++it)
        {

            tf2::Quaternion quat_tf;
            tf2::convert(it->second.orientation, quat_tf);

            std::array<double, 3> rpy;

            rpy = utils_ptr_->set_euler_from_quaternion(quat_tf);

            std::cout << it->first << " detected at "
                      << "xyz=[" << std::round(it->second.position.x * 1000) / 1000.0 << " "
                      << std::round(it->second.position.y * 1000) / 1000.0 << " "
                      << std::round(it->second.position.z * 1000) / 1000.0 << "] rpy=["
                      << std::round(rpy.at(0) * 1000) / 1000.0
                      << " " << std::round(rpy.at(1) * 1000) / 1000.0
                      << " " << std::round(rpy.at(2) * 1000) / 1000.0 << "]\n\n";
        }

        rclcpp::shutdown();
    }

    // Checks if state of robot is move forward
    if (isStraight == true)
    {
        // calculate distance between robot and aruco
        d_robot_aruco = compute_distance(robot_position, aruco_position);

        if (d_robot_aruco < 0.9)
        {
            cmd_vel.linear.x = 0.0;
            isTurning = true;
            isStraight = false;
            initialYaw = robot_orientation_euler.z;
            update_aruco_target();
        }

        else
        {
            cmd_vel.linear.x = 0.1;
            isTurning = false;
            isStraight = true;
        }
    }

    // Incase state is turning, turn to 90 degrees
    if (isTurning == true)
    {
        double angle_turned = std::abs(std::abs(initialYaw) - std::abs(robot_orientation_euler.z));

        if (angle_turned < 1.54) // 0.03 offset to compensate message delays
        {
            cmd_vel.angular.z = 0.1 * turning_direction;
        }

        else
        {
            cmd_vel.angular.z = 0;
            isTurning = false;
            isStraight = true;
        }
        // RCLCPP_INFO_STREAM(this->get_logger(), "==============================================================");
        // RCLCPP_INFO_STREAM(this->get_logger(), "Angle turned " << angle_turned << " radians turn dir " << turning_direction);
        // RCLCPP_INFO_STREAM(this->get_logger(), "Initial Yaw " << std::abs(initialYaw) << " Robot Z " << std::abs(robot_orientation_euler.z));
    }

    // publish the velocity command
    cmd_vel_publisher_->publish(cmd_vel);
}

//==============================================================================
/**
 * compute distance b/w 2 (x,y) points
 */
double RobotController::compute_distance(const geometry_msgs::msg::Point &robot_position, const geometry_msgs::msg::Point &aruco_position)
{
    // Calculate the differences in x, y, and z coordinates
    double dx = robot_position.x - aruco_position.x;
    double dy = robot_position.y - aruco_position.y;

    // Compute the Euclidean distance
    double distance = std::sqrt(dx * dx + dy * dy);

    return distance;
}

//==============================================================================
/**
 * based on aruco data, set the direction of turn
 */
void RobotController::update_aruco_target()
{

    turning_direction = (aruco_map[target_aruco] == "right_90") ? -1 : (aruco_map[target_aruco] == "left_90") ? 1
                                                                                                              : 0;
    // RCLCPP_INFO_STREAM(this->get_logger(), "Current Aruco Target " << target_aruco << " command " << aruco_map[target_aruco]);
}

//==============================================================================
/**
 * Get detected_parts TF data
 */
void RobotController::listen_parts_transform()
{
    if (detected_part != "")
    {
        std::string target_frame = detected_part;
        std::string source_frame = "odom";
        geometry_msgs::msg::TransformStamped t_stamped;
        geometry_msgs::msg::Pose pose_out;
        try
        {
            t_stamped = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 50ms);
        }
        catch (const tf2::TransformException &ex)
        {
            // RCLCPP_WARN_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
            return;
        }

        pose_out.position.x = t_stamped.transform.translation.x;
        pose_out.position.y = t_stamped.transform.translation.y;
        pose_out.position.z = t_stamped.transform.translation.z;
        pose_out.orientation = t_stamped.transform.rotation;

        geometry_msgs::msg::Pose part_pose;
        part_pose.position = pose_out.position;
        part_pose.orientation = pose_out.orientation;

        if (detected_parts.find(detected_part) != detected_parts.end())
        {
            ;
        }
        else
        {
            RCLCPP_INFO_STREAM(this->get_logger(), detected_part << " New part found"
                                                                 << ":\n");
            detected_parts[detected_part] = part_pose;
        }
    }
}

//==============================================================================
/**
 * Get message notifying a detected part
 */
void RobotController::parts_callback(const std_msgs::msg::String::SharedPtr msg)
{
    detected_part = msg->data;
    // RCLCPP_INFO_STREAM(this->get_logger(), "Part detected " << msg->data);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>("robot_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
}