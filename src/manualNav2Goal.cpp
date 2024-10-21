#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <chrono>
#include <cmath> 
#include <iostream>
 // We include everything about OpenCV as we don't care much about compilation time at the moment.

using std::placeholders::_1;
using namespace std::chrono_literals;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

enum dirEnum {
  NORTH,
  SOUTH,
  EAST,
  WEST
}; 

//dummy change

struct goalStruct {
    double xPos = 0;
    double yPos = 0;
    dirEnum dir = NORTH;
};
 
class manualNav2Goal : public rclcpp::Node {
public:
manualNav2Goal() : Node("manualNav2GoalNode"){    
    std::cout << "manualNav2Goal started node" << std::endl;

    // Initialize the action client
    client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Wait until the action server is available
    while (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))){
            RCLCPP_INFO(this->get_logger(), "Waiting for the action server to be available...");
    }

    timer_ = this->create_wall_timer(500ms, std::bind(&manualNav2Goal::timer_callback, this));
    // this->send_goal(0,0,SOUTH);
    goalStruct g1;
    g1.dir = EAST;
    g1.xPos = 0;
    g1.yPos = -1;

    goalStruct g2;
    g2.dir = NORTH;
    g2.xPos = 1.1;
    g2.yPos = 2;

    goalStruct g3;
    g3.dir = EAST;
    g3.xPos = 1;
    g3.yPos = -1;

    goalStruct g4;
    g4.dir = SOUTH;
    g4.xPos = -2;
    g4.yPos = 7;

    goalStruct g5;
    g5.dir = NORTH;
    g5.xPos = 0;
    g5.yPos = 0;

    goals_.push_back(g1);
    goals_.push_back(g2);
    goals_.push_back(g3);
    goals_.push_back(g4);
    goals_.push_back(g5);

    goalTargetted_ = false;
    currentGoalIndex_ = 0;
  }

private:
    void send_goal(goalStruct goal){
        goalTargetted_ = true;
        // Create a goal message with a target pose
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";  // Use 'map' as the reference frame
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = goal.xPos;  // Set the desired goal position
        goal_msg.pose.pose.position.y = goal.yPos;

        goal_msg.pose.pose.orientation.x = 0;
        goal_msg.pose.pose.orientation.y = 0;
        
        switch (goal.dir){
        case NORTH:
            goal_msg.pose.pose.orientation.z = 0;
            goal_msg.pose.pose.orientation.w = 1;
            break;
        case SOUTH:
            goal_msg.pose.pose.orientation.z = 1;
            goal_msg.pose.pose.orientation.w = 0;
            break;
        case EAST:
            goal_msg.pose.pose.orientation.z = 0.7071;
            goal_msg.pose.pose.orientation.w = 0.7071;
            break;
        case WEST:
            goal_msg.pose.pose.orientation.z = -0.7071;
            goal_msg.pose.pose.orientation.w = 0.7071;
            break;
            
        default:
            break;
        }

        // Send the goal to the server
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult & result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                goalTargetted_ = false;
                currentGoalIndex_ += 1;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Goal failed with result code: %d", result.code);
            }
        };

        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void timer_callback(){
        if(!goalTargetted_ && currentGoalIndex_ < goals_.size()){
            std::cout << "Sending new Goal" << std::endl;
            send_goal(goals_.at(currentGoalIndex_));
        }
    }
 
private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    bool goalTargetted_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<goalStruct> goals_;
    int currentGoalIndex_;
};
 
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<manualNav2Goal>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}