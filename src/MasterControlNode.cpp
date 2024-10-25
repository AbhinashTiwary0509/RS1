#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <chrono>
#include <cmath> 
#include <iostream>

using std::placeholders::_1;
using namespace std::chrono_literals;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

struct goalStruct {
    double xPos = 0;
    double yPos = 0;
    double yaw = 0; //yaw of the goal
};

struct quartRotXYZW {
    double x = 0;
    double y = 0;
    double z = 0;
    double w = 0;
};

class robot {
public:
    robot(){
        std::cout << "Intialised robot" << std::endl;

        //initialise default values for the robot
        busy_ = false;

        goalStruct defaultGoal;
        defaultGoal.xPos = 0;
        defaultGoal.yPos = 0;
        defaultGoal.yaw = 0;
        currentGoal_ = defaultGoal;

        ID_ = 0;
    }

    bool getAvailability(){
        return busy_;
    }
    
    goalStruct getCurrentGoal(){
        return currentGoal_;
    }

    void setRobotID(int ID){
        ID_ = ID;
    }

    int getRobotID(){
        return ID_;
    }

    void setGoal(goalStruct goal){
        currentGoal_ = goal;
        busy_ = true;
    }

    void flagGoalAsAchieved(){
        busy_ = false;
    }

private:
    bool busy_;
    goalStruct currentGoal_;
    int ID_;
};
 
class MasterControlNode : public rclcpp::Node {
public:
MasterControlNode() : Node("MasterControlNode"){    
    std::cout << "MasterControlNode started" << std::endl;

    programStartXPos_ = 0;
    programStartYPos_ = 0;
    programStartYaw_ = 0;

    // Initialize the action client
    client_ptr_NAV2POSE = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Wait until the action server is available
    while (!client_ptr_NAV2POSE->wait_for_action_server(std::chrono::seconds(5))){
            RCLCPP_INFO(this->get_logger(), "Waiting for the action server to be available...");
    }

    //instial pose publisher
    initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
    publish_initial_pose();

    for(int i = 0; i < 10; i++){
        publish_initial_pose();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));

    goalAssignerTimer_ = this->create_wall_timer(1000ms, std::bind(&MasterControlNode::goalAssignerTimer_callback, this)); //timer runs every second and decides if goals should be assigned

    addGoalToQueue(0, 10, -2);
    addGoalToQueue(0, 1.1, 2);
    addGoalToQueue(0, 1, -2);
    addGoalToQueue(0, -2, 7);

    goalTargetted_ = false;
    currentGoalIndex_ = 0;

    //intialise robot
    robot robotInstance;
    robotInstance.setRobotID(0);
    robots_.push_back(robotInstance);

    targetRobotID_ = 0;
  }

private:
    void addGoalToQueue(double xPos, double yPos, double yaw){
        goalStruct newGoal;
        newGoal.xPos = xPos;
        newGoal.yPos = yPos;
        newGoal.yaw = yaw;
        goalQueue_.push_back(newGoal);
    }

    quartRotXYZW yawToXYZW(double yaw){
        quartRotXYZW ret;
        //x and y default to 0
        ret.z = std::sin(yaw / 2.0);
        ret.w = std::cos(yaw / 2.0);
        return ret;
    }

    void sendGoal(goalStruct goal){
        goalTargetted_ = true;
        // Create a goal message with a target pose
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";  // Use 'map' as the reference frame
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = goal.xPos;  // Set the desired goal position
        goal_msg.pose.pose.position.y = goal.yPos;

        //quarternion for yaw
        quartRotXYZW contXYZW = yawToXYZW(goal.yaw);
        goal_msg.pose.pose.orientation.x = contXYZW.x;
        goal_msg.pose.pose.orientation.y = contXYZW.y;
        goal_msg.pose.pose.orientation.z = contXYZW.z;
        goal_msg.pose.pose.orientation.w = contXYZW.w;

        // Send the goal to the server
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult & result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                robots_.at(targetRobotID_).flagGoalAsAchieved();
                goalTargetted_ = false;
                currentGoalIndex_ += 1;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Goal failed with result code: %d", result.code);
            }
        };

        client_ptr_NAV2POSE->async_send_goal(goal_msg, send_goal_options);
    }

    void goalAssignerTimer_callback(){
        //add logic for choosing robot ID for given tasks
        targetRobotID_ = 0;

        if(!goalTargetted_ && currentGoalIndex_ < goalQueue_.size()){
            std::cout << "Sending new Goal" << std::endl;
            sendGoal(goalQueue_.at(currentGoalIndex_));
            robots_.at(targetRobotID_).setGoal(goalQueue_.at(currentGoalIndex_));
        }
    }

    void publish_initial_pose() {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.frame_id = "map";  // Frame of reference
        pose_msg.header.stamp = this->get_clock()->now();

        quartRotXYZW XYZW = yawToXYZW(programStartYaw_);

        // Define the initial position and orientation
        pose_msg.pose.pose.position.x = programStartXPos_;  // Example x position
        pose_msg.pose.pose.position.y = programStartYPos_;  // Example y position
        
        pose_msg.pose.pose.orientation.x = XYZW.x;
        pose_msg.pose.pose.orientation.y = XYZW.y;
        pose_msg.pose.pose.orientation.z = XYZW.z;
        pose_msg.pose.pose.orientation.w = XYZW.w;

        // Optionally set covariance values
        pose_msg.pose.covariance[0] = 0.0;   // Covariance on x
        pose_msg.pose.covariance[7] = 0.0;   // Covariance on y
        pose_msg.pose.covariance[35] = 0.0; // Covariance on yaw

        // Publish the message
        initial_pose_publisher_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "Published initial pose estimate");
    }
 
private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_NAV2POSE;
    bool goalTargetted_;
    rclcpp::TimerBase::SharedPtr goalAssignerTimer_;
    std::vector<goalStruct> goalQueue_;
    int currentGoalIndex_;
    std::vector<robot> robots_;
    int targetRobotID_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;

    double programStartXPos_;
    double programStartYPos_;
    double programStartYaw_;

};


 
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MasterControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}