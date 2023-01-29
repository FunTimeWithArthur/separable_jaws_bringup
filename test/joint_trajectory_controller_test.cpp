/**
 * @file joint_trajectory_controller_test.cpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Test for testing the controllability of separable_jaws, especially.
 */

// std include
#include <chrono>
#include <memory>
#include <functional>
#include <future>
#include <stdexcept>
#include <string>
#include <vector>

// ros2 include
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// ros2 interface include
#include "control_msgs/action/follow_joint_trajectory.hpp"

static const std::vector<std::string> left_arm_join_name = {
    "left_shoulder_joint",
    "left_upper_arm_joint",
    "left_lower_arm_joint",
    "left_wrist_joint",
    "left_palm_joint"
};

static const std::vector<std::string> right_arm_join_name = {
    "right_shoulder_joint",
    "right_upper_arm_joint",
    "right_lower_arm_joint",
    "right_wrist_joint",
    "right_palm_joint"
};

static const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> left_arm_join_points = []() {
    trajectory_msgs::msg::JointTrajectoryPoint point1;
    point1.time_from_start = rclcpp::Duration::from_seconds(0.0);  // start asap
    point1.positions.assign(left_arm_join_name.size(), 0.0);

    trajectory_msgs::msg::JointTrajectoryPoint point2;
    point2.time_from_start = rclcpp::Duration::from_seconds(2.0);
    point2.positions.assign(left_arm_join_name.size(), -6.28);

    trajectory_msgs::msg::JointTrajectoryPoint point3;
    point3.time_from_start = rclcpp::Duration::from_seconds(4.0);
    point3.positions.assign(left_arm_join_name.size(), 6.28);

    trajectory_msgs::msg::JointTrajectoryPoint point4;
    point4.time_from_start = rclcpp::Duration::from_seconds(6.0);
    point4.positions.assign(left_arm_join_name.size(), 0.0);

    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points = {point1, point2, point3, point4};
    return points;
}();

static const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> right_arm_join_points = []() {
    trajectory_msgs::msg::JointTrajectoryPoint point1;
    point1.time_from_start = rclcpp::Duration::from_seconds(0.0);  // start asap
    point1.positions.assign(right_arm_join_name.size(), 0.0);

    trajectory_msgs::msg::JointTrajectoryPoint point2;
    point2.time_from_start = rclcpp::Duration::from_seconds(2.0);
    point2.positions.assign(right_arm_join_name.size(), 6.28);

    trajectory_msgs::msg::JointTrajectoryPoint point3;
    point3.time_from_start = rclcpp::Duration::from_seconds(4.0);
    point3.positions.assign(right_arm_join_name.size(), -6.28);

    trajectory_msgs::msg::JointTrajectoryPoint point4;
    point4.time_from_start = rclcpp::Duration::from_seconds(6.0);
    point4.positions.assign(right_arm_join_name.size(), 0.0);

    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points = {point1, point2, point3, point4};
    return points;
}();

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for testing the controllability of separable_jaws, especially.
 */
class JointTrajectoryControllerTest : public rclcpp::Node {
    public:
        /// @brief Constructor of fake_socket_can_bridge_node.
        JointTrajectoryControllerTest(rclcpp::NodeOptions _options) : Node("joint_trajectory_controller_test_node", _options),
            left_arm_trajectory_clt_(rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(this,
                "/left_arm_controller/follow_joint_trajectory")),
            right_arm_trajectory_clt_(rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(this,
                "/right_arm_controller/follow_joint_trajectory")),
            trajectory_test_timer_(this->create_wall_timer(std::chrono::seconds(10),
                std::bind(&JointTrajectoryControllerTest::trajectory_test_callback, this))),
            
            using_left_arm_(this->declare_parameter("using_left_arm", true)),
            using_right_arm_(this->declare_parameter("using_right_arm", true)) {

            // check if both trajectory controller server exist
            if (!left_arm_trajectory_clt_->wait_for_action_server(std::chrono::seconds(1))) {
                throw std::runtime_error("Could not get action server \"/left_arm_controller/follow_joint_trajectory\".");
            }
            if (!right_arm_trajectory_clt_->wait_for_action_server(std::chrono::seconds(1))) {
                throw std::runtime_error("Could not get action server \"/right_arm_controller/follow_joint_trajectory\".");
            }
            RCLCPP_INFO(this->get_logger(), "Created action server: \"%s\", \"%s\".",
                "/left_arm_controller/follow_joint_trajectory", "/right_arm_controller/follow_joint_trajectory");

            trajectory_test_timer_->execute_callback();
        }
    private:
        /// @brief ROS2 client to "/left_arm_controller/follow_joint_trajectory", for sending test joint trajectory action to the left arm.
        rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr left_arm_trajectory_clt_;

        /// @brief ROS2 client to "/right_arm_controller/follow_joint_trajectory", for sending test joint trajectory action to the right arm.
        rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr right_arm_trajectory_clt_;

        /// @brief ROS2 timer for periodically testing joint_trajectory_controller.
        rclcpp::TimerBase::SharedPtr trajectory_test_timer_;

        // internal status
        //// @brief Flag to determine if to use left arm.
        bool using_left_arm_;

        //// @brief Flag to determine if to use right arm.
        bool using_right_arm_;
        
        /// @brief Timed callback function for periodically testing joint_trajectory_controller.
        void trajectory_test_callback() {
            if(using_left_arm_) {
                control_msgs::action::FollowJointTrajectory_Goal goal_msg;
                goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
                goal_msg.trajectory.joint_names = std::move(left_arm_join_name);
                goal_msg.trajectory.points = left_arm_join_points;

                rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions action_opt;
                action_opt.goal_response_callback = std::bind(&JointTrajectoryControllerTest::onActionGoalResponse,
                    this, std::placeholders::_1);
                action_opt.result_callback = std::bind(&JointTrajectoryControllerTest::onActionResultResponse,
                    this, std::placeholders::_1);
                action_opt.feedback_callback = std::bind(&JointTrajectoryControllerTest::onActionFeedback,
                    this, std::placeholders::_1, std::placeholders::_2);

                left_arm_trajectory_clt_->async_send_goal(goal_msg, action_opt);
            }

            if(using_right_arm_) {
                control_msgs::action::FollowJointTrajectory_Goal goal_msg;
                goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
                goal_msg.trajectory.joint_names = std::move(right_arm_join_name);
                goal_msg.trajectory.points = right_arm_join_points;

                rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions action_opt;
                action_opt.goal_response_callback = std::bind(&JointTrajectoryControllerTest::onActionGoalResponse,
                    this, std::placeholders::_1);
                action_opt.result_callback = std::bind(&JointTrajectoryControllerTest::onActionResultResponse,
                    this, std::placeholders::_1);
                action_opt.feedback_callback = std::bind(&JointTrajectoryControllerTest::onActionFeedback,
                    this, std::placeholders::_1, std::placeholders::_2);

                right_arm_trajectory_clt_->async_send_goal(goal_msg, action_opt);
            }
        }

        /// @brief Callback function when action server responds to the goal.
        void onActionGoalResponse(rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr _goal_handle) {
            RCLCPP_DEBUG(this->get_logger(), "Trajectory controller goal response time: %f", rclcpp::Clock().now().seconds());
            if (!_goal_handle) {
                RCLCPP_FATAL(this->get_logger(), "Goal rejected");
            }
            else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted");
            }
        }

        /// @brief Callback function when action server finishes the goal.
        void onActionResultResponse(const rclcpp_action::ClientGoalHandle <control_msgs::action::FollowJointTrajectory>::WrappedResult &_result) {
            RCLCPP_DEBUG(this->get_logger(), "Trajectory controller goal response time: %f\n", rclcpp::Clock().now().seconds());
            switch (_result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Goal succeed");
                    break;

                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_INFO(this->get_logger(), "Goal was aborted");
                    return;

                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_INFO(this->get_logger(), "Goal was canceled");
                    return;

                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    return;
            }
        }

        /// @brief Callback function when action server feeds back.
        void onActionFeedback(rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
            const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> _feedback) {

            std::string desired_position_str, actual_position_str;
            for(auto &desired_position : _feedback->desired.positions) {
                desired_position_str += std::to_string(desired_position) + " ";
            }
            for(auto &actual_position : _feedback->actual.positions) {
                actual_position_str += std::to_string(actual_position) + " ";
            }

            RCLCPP_INFO(this->get_logger(), "Feedback:\ndesired positions: %s\nactual positions: %s",
                desired_position_str.c_str(), actual_position_str.c_str());
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::executors::StaticSingleThreadedExecutor executor;
    rclcpp::NodeOptions options;

    auto joint_trajectory_controller_test_node = std::make_shared<JointTrajectoryControllerTest>(options);

    executor.add_node(joint_trajectory_controller_test_node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
