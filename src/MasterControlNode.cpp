#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <chrono>
#include <cmath> 
#include <iostream>
#include <optional>

#include "SimpleGUI.hpp"
#include <QApplication>
#include "PDFgenerator.cpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

enum robotStatus{
    WAITING = 0,
    INPUT_TARGET,
    OUTPUT_TARGET,
    WAITING_WITH_OBJECT,
    OTHER_R,
};

enum goalType{
    INPUT_GOAL = 0,
    OUTPUT_GOAL,
    OTHER,
};

struct goalStruct {
    double xPos = 0;
    double yPos = 0;
    double yaw = 0; //yaw of the goal
    goalType type = OTHER;
    bool nullified = false;

    bool operator==(const goalStruct& other) const {
    return (xPos == other.xPos) &&
            (yPos == other.yPos) &&
            (yaw == other.yaw) &&
            (type == other.type);
    }

    void nullify() {
        xPos = 0;
        yPos = 0;
        yaw = 0;
        type = OTHER; // Assuming you want to reset type to OTHER as well
        nullified = true;
    }
};

struct ObjectStruct {
    double xPos = 0;
    double yPos = 0;
    int ID = 0;
    bool targeted = false;
    bool stored = false;

    std::string toString() const {
        std::stringstream ss;
        ss << "Object at "
           << " xPos: " << xPos
           << ",  yPos: " << yPos
           << ", with ID: " << ID 
           << ", targeted: " << (targeted ? "true" : "false")
           << ",  stored: " << (stored ? "true" : "false");
        return ss.str();
    }
};

struct quartRotXYZW {
    double x = 0;
    double y = 0;
    double z = 0;
    double w = 0;
};

struct storagePosition{
    goalStruct position;
    bool empty = true;

    std::string toString() const {
    std::stringstream ss;
    ss << "Storage Position at "
        << " xPos: " << position.xPos
        << ",  yPos: " << position.yPos
        << ", empty: " << (empty ? "true" : "false");
    return ss.str();
}
};

struct robotData{
    int numberOfGoalsAcheived = 0;
    int numberOfObjectsMoved = 0;
    robotStatus robStatus = WAITING;
};

struct storageReport{
    int available = 0;
    int full = 0;
    int objectsS = 0;
    int objectsUS = 0; 
};

struct dailyReportData{
    int numberOfRobots = 0;
    std::vector<robotData> robotDataVector;
    storageReport storageDaily;
};

class ObjectStorage {
public:
    ObjectStorage(){
        //where the objects are at the start of the run
        //example inout area object
        createObject(0, 1.6, -0.8);
        createObject(1, 1, -0.8);
        createObject(2, 0.4, -0.8);
        
        //create all of the valid storage positions
        addStoragePosition(1.6, 1.4, deg2rad(180));
        addStoragePosition(1.6, 2.1, deg2rad(180));
        addStoragePosition(1.6, 3, deg2rad(180));
        addStoragePosition(1.6, 4, deg2rad(180));
        addStoragePosition(1.6, 6.4, deg2rad(180));
        addStoragePosition(1.6, 7.1, deg2rad(180));
        addStoragePosition(1.6, 7.8, deg2rad(180));
        addStoragePosition(1.6, 9, deg2rad(180));
        addStoragePosition(0.45, 1.4, 0);
        addStoragePosition(0.45, 2.1, 0);
        addStoragePosition(0.45, 3, 0);
        addStoragePosition(0.45, 4, 0);
        addStoragePosition(0.45, 6.4, 0);
        addStoragePosition(0.45, 7.1, 0);
        addStoragePosition(0.45, 7.8, 0);
        addStoragePosition(0.45, 9, 0);
        addStoragePosition(-0.45, 1.4, deg2rad(180));
        addStoragePosition(-0.45, 2.1, deg2rad(180));
        addStoragePosition(-0.45, 3, deg2rad(180));
        addStoragePosition(-0.45, 4, deg2rad(180));
        addStoragePosition(-0.45, 6.4, deg2rad(180));
        addStoragePosition(-0.45, 7.1, deg2rad(180));
        addStoragePosition(-0.45, 7.8, deg2rad(180));
        addStoragePosition(-0.45, 9, deg2rad(180));
        addStoragePosition(-1.8, 1.4, 0);
        addStoragePosition(-1.8, 2.1, 0);
        addStoragePosition(-1.8, 3, 0);
        addStoragePosition(-1.8, 4, 0);
        addStoragePosition(-1.8, 6.4, 0);
        addStoragePosition(-1.8, 7.1, 0);
        addStoragePosition(-1.8, 7.8, 0);
        addStoragePosition(-1.8, 9, 0);

        PDF_LIVE_.updateOutputFileName("LIVE.pdf");
        // generatePDF(false);//live call

        PDF_DAILY_.updateOutputFileName("DAILY.pdf");
        // generatePDF(true);//daily call

    }

    storageReport generateStorageData(){
        storageReport ret;

        //calculate data
        for(size_t i = 0; i<storagePositions_.size(); i++){
            if(storagePositions_.at(i).empty == true){
                ret.available++;
            }else{
                ret.full++;
            }
        }

        for(size_t i = 0; i<objects_.size(); i++){
            if(objects_.at(i).stored == true){
                ret.objectsS++;
            }else{
                ret.objectsUS++;
            }
        }

        return ret;
    }

    void generatePDF(bool dailyOrLive, std::optional<dailyReportData> DRData = std::nullopt){ //daily or live report
        if(dailyOrLive){ //daily report
            try {
                PDF_DAILY_.clear();     

                PDF_DAILY_.addText(50, 775, "DAILY REPORT");  //title

                int textX= 60;
                int textY = 750;
                int textYinc = 15;
                
                PDF_DAILY_.addText(textX, textY, "Number of Robots: " + std::to_string(DRData->numberOfRobots));
                textY -= textYinc;

                for(size_t i = 0; i<DRData->robotDataVector.size(); i++){
                    PDF_DAILY_.addText(textX, textY, "For Robot ID: " + std::to_string(i));
                    textY -= textYinc;
                    PDF_DAILY_.addText(textX, textY, "Current robot status: " + std::to_string(DRData->robotDataVector.at(i).robStatus));
                    textY -= textYinc;
                    PDF_DAILY_.addText(textX, textY, "Number of goals achieved: " + std::to_string(DRData->robotDataVector.at(i).numberOfGoalsAcheived));
                    textY -= textYinc;
                    PDF_DAILY_.addText(textX, textY, "Number objects moved: " + std::to_string(DRData->robotDataVector.at(i).numberOfObjectsMoved));
                    textY -= textYinc;
                }

                PDF_DAILY_.addText(textX, textY, "Storage Data: ");
                textY -= textYinc;
                PDF_DAILY_.addText(textX, textY, "Available Storage Positions: " + std::to_string(DRData->storageDaily.available));
                textY -= textYinc;
                PDF_DAILY_.addText(textX, textY, "Full Storage Positions: " + std::to_string(DRData->storageDaily.full));
                textY -= textYinc;
                PDF_DAILY_.addText(textX, textY, "Objects Stored: " + std::to_string(DRData->storageDaily.objectsS));
                textY -= textYinc;
                PDF_DAILY_.addText(textX, textY, "Objects Unstored " + std::to_string(DRData->storageDaily.objectsUS));

                PDF_DAILY_.save("DAILY_REPORT.pdf");
                std::cout << "Daily Report PDF generated" << std::endl;
            }catch (const std::exception& e){
                std::cerr << "Error: " << e.what() << std::endl;
            }
        }else{ //live report
            try {
                PDF_LIVE_.clear();     
                // Add some sample data
                std::string tempString;
                int textX= 50;
                int textY = 750;
                int textYinc = 15;
                PDF_LIVE_.addText(50, 775, "LIVE REPORT");  //title
                for(size_t i = 0; i < objects_.size(); i++){
                    tempString = objects_.at(i).toString();
                    std::cout << tempString << std::endl;
                    PDF_LIVE_.addText(textX, textY, tempString);
                    textY -= textYinc;
                }

                for(size_t i = 0; i < storagePositions_.size(); i++){
                    tempString = storagePositions_.at(i).toString();
                    std::cout << tempString << std::endl;
                    PDF_LIVE_.addText(textX, textY, tempString);
                    textY -= textYinc;
                }

                PDF_LIVE_.save("LIVE_REPORT.pdf");
                std::cout << "LIVE_REPORT PDF generated" << std::endl;
            }
            catch (const std::exception& e) {
                std::cerr << "Error generating LIVE_REPORT: " << e.what() << std::endl;
            }
        }
        std::cout << "Leaving generate PDF" << std::endl;
    }

    void createObject(int ID, double xPos, double yPos){
        ObjectStruct newObject;
        newObject.xPos = xPos;
        newObject.yPos = yPos;
        newObject.ID = ID;

        //append to the list of objects in input zone
        objects_.push_back(newObject);
    }

    void targetObject(int ID, bool toggle){ //toggle = true: make targetted. = false: untarget
        int indexTarget = -1; //default
        for(size_t i = 0; i < objects_.size(); i++){
            if(objects_.at(i).ID == ID){
                indexTarget = i;
            }
        }

        if(indexTarget == -1){
            std::cout << "ERR_ storage object ID not found" << std::endl;
        }else{
            if(toggle){ //target
                objects_.at(indexTarget).targeted = true;
            }else{//untarget
                objects_.at(indexTarget).targeted = false;
            }
        }
    }

    bool generateGoalForObjectPickup(goalStruct &goal, int &Obj_ID){ //true if there is a goal false otherwise, will pass the goal by reference
        int indexTarget = -1;
        for(size_t i = 0; i < objects_.size(); i++){
            if(objects_.at(i).stored == false && objects_.at(i).targeted == false){
                indexTarget = i;
            }
        }
        if(indexTarget == -1){
            std::cout << "No object to target" << std::endl;
            return false;
        }else{
            objects_.at(indexTarget).stored = true;
            Obj_ID = objects_.at(indexTarget).ID;
            goal.xPos = objects_.at(indexTarget).xPos;
            goal.yPos = objects_.at(indexTarget).yPos;
            goal.type = INPUT_GOAL;
            goal.yaw = 90*(M_PI/180); //default yaw for target is 0 change later to fit input area
            objects_.at(indexTarget).targeted = true;
            return true;
        }
        std::cout << "" << std::endl;
        return false; //default
    }

    bool getAvailableStoragePosition(goalStruct &goal){ //true for storage available false for no storage available
        for(size_t i = 0; i< storagePositions_.size(); i++){
            if(storagePositions_.at(i).empty == true){
                storagePositions_.at(i).empty = false; //if this function is called to get this position it is automatically considered full
                goal = storagePositions_.at(i).position;
                return true;
            }
        }
        std::cout << "ERR_ No Storage Positions available" << std::endl;
        return false;
    }

private:
    double deg2rad(double deg){
        return deg * M_PI / 180.0;
    }

    void addStoragePosition(double xPos, double yPos, double yaw){
        goalStruct newGoal;
        newGoal.xPos = xPos;
        newGoal.yPos = yPos;
        newGoal.yaw = yaw;

        storagePosition newStoragePosition;
        newStoragePosition.position = newGoal;
        newStoragePosition.empty = true;

        //add to storage positions
        storagePositions_.push_back(newStoragePosition);
    }

    std::vector<ObjectStruct> objects_;
    std::vector<storagePosition> storagePositions_;
    PDFGenerator PDF_DAILY_;
    PDFGenerator PDF_LIVE_;
};

class robot {
public:
    robot(){
        std::cout << "Intialised robot" << std::endl;

        //initialise default values for the robot
        status_ = WAITING;

        goalStruct defaultGoal;
        defaultGoal.xPos = 0;
        defaultGoal.yPos = 0;
        defaultGoal.yaw = 0;
        currentGoal_ = defaultGoal;

        robotID_ = 0;

        heldObjectID_ = -1;
        objectHeld_ = false;

        totalObjectsMoved_ = 0;
        totalGoalsReached_ = 0;
    }

    int getStatus(){
        return status_;
    }
    
    goalStruct getCurrentGoal(){
        return currentGoal_;
    }

    void setRobotID(int ID){
        robotID_ = ID;
    }

    int getRobotID(){
        return robotID_;
    }

    void setGoal(goalStruct goal){ //inputOutput true for input area goal, false for storage goal
        currentGoal_ = goal;
        if(goal.type ==INPUT_GOAL){
            status_ = INPUT_TARGET;
        }else if(goal.type == OUTPUT_GOAL){
            status_ = OUTPUT_TARGET;
        }else{
            status_ = OTHER_R;
        }
    }

    void flagGoalAsAchieved(){
        if(heldObjectID_ != -1){
            status_ = WAITING_WITH_OBJECT;
            std::cout << "[ROBOT] status: WAITING_WITH_OBJECT" << std::endl;
        }else{
            status_ = WAITING;
            std::cout << "[ROBOT] status: WAITING" << std::endl;
        }
        totalGoalsReached_++;
    }

    void pickupDropoffObjectToggle(bool pickupDropoff, int ID = -1){ //true for object being picked up false otherwise
        if(pickupDropoff){
            objectHeld_ = true;
            std::cout << "[ROBOT] picking up object" << std::endl;
            heldObjectID_ = ID;
            totalObjectsMoved_++;
        }else{
            objectHeld_ = false;
            std::cout << "[ROBOT] dropping off object" << std::endl;
            heldObjectID_= -1;
        }
    }

    robotData getRobotData(){
        robotData stat;
        stat.numberOfGoalsAcheived = totalGoalsReached_;
        stat.numberOfObjectsMoved = totalObjectsMoved_;
        stat.robStatus = status_;
        return stat;
    }

private:
    robotStatus status_;
    goalStruct currentGoal_;
    int robotID_;
    int heldObjectID_;
    bool objectHeld_;
    
    //record keeping
    int totalObjectsMoved_;
    int totalGoalsReached_;

};
 
class MasterControlNode : public rclcpp::Node {
public:
MasterControlNode() : Node("MasterControlNode"){    
    std::cout << "MasterControlNode started" << std::endl;

    //initialise object storage
    // storage_ = ObjectStorage();//dont think this is needed as it gets intialised anyway?

    //testing########################
    // Initialize GUI
    init_gui();
    // //testing###############


    programStartXPos_ = 0;
    programStartYPos_ = 0;
    programStartYaw_ = 0;
    currentGoalSent_ = false;
    testingCount_ = 0;

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

    //timer used to assign the current goal
    goalAssignerTimer_ = this->create_wall_timer(1000ms, std::bind(&MasterControlNode::goalManagerTimer_callback, this)); //timer runs every second and decides if goals should be assigned

    //timer used to call the blocking service call
    serviceManagerTimer_ = this->create_wall_timer(1000ms, std::bind(&MasterControlNode::serviceManagerTimer_callback, this));

    checkButtonTimer = this->create_wall_timer(100ms, std::bind(&MasterControlNode::checkButtonClicked, this));

    //intialise robot
    robot robotInstance;
    robotInstance.setRobotID(0);
    robots_.push_back(robotInstance);
    targetRobotID_ = 0;
  }

private:

    void init_gui(){
        // Initialize GUI
        static int argc = 0;
        static char** argv = nullptr;
        app_ = std::make_unique<QApplication>(argc, argv);
        
        // Create GUI with properly constructed shared_ptr
        auto node_ptr = std::shared_ptr<MasterControlNode>(this, [](MasterControlNode*){});
        gui_ = new SimpleGUI(node_ptr);
        gui_->show();
        
        // Start Qt event loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() {
                app_->processEvents();
            });
    }

    quartRotXYZW yawToXYZW(double yaw){
        quartRotXYZW ret;
        //x and y default to 0
        ret.z = std::sin(yaw / 2.0);
        ret.w = std::cos(yaw / 2.0);
        return ret;
    }

    void sendGoal(goalStruct goal){
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

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        // Add a goal response callback to store the handle
        send_goal_options.goal_response_callback =
            [this](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
                    this->current_goal_handle_ = goal_handle;
                }
            };

        // Send the goal to the server
        send_goal_options.result_callback =
            [this](const GoalHandleNavigateToPose::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Goal COMPLETE");
                        robots_.at(targetRobotID_).flagGoalAsAchieved();
                        currentGoal_.nullify();
                        this->current_goal_handle_ = nullptr;  // Clear the handle
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_INFO(this->get_logger(), "Goal was aborted");
                        currentGoal_.nullify();
                        this->current_goal_handle_ = nullptr;  // Clear the handle
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_INFO(this->get_logger(), "Goal was canceled");
                        currentGoal_.nullify();
                        this->current_goal_handle_ = nullptr;  // Clear the handle
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                        this->current_goal_handle_ = nullptr;  // Clear the handle
                        break;
                }
            };

        client_ptr_NAV2POSE->async_send_goal(goal_msg, send_goal_options);
    }

    void cancelCurrentGoal() {
        if (current_goal_handle_ != nullptr) {
            RCLCPP_INFO(this->get_logger(), "Canceling current goal");
            
            // Create a callback for the cancel response
            auto cancel_response_callback = [this](std::shared_ptr<action_msgs::srv::CancelGoal_Response> response) {
                if (response->return_code == action_msgs::srv::CancelGoal_Response::ERROR_NONE) {
                    RCLCPP_INFO(this->get_logger(), "Goal successfully canceled");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to cancel goal");
                }
            };

            // Send the cancel request asynchronously
            client_ptr_NAV2POSE->async_cancel_goal(current_goal_handle_, cancel_response_callback);
            
        } else {
            RCLCPP_WARN(this->get_logger(), "No active goal to cancel");
        }
    }

    void serviceManagerTimer_callback(){
        //if the goal is not nullified
        if(currentGoal_.nullified == false && currentGoalSent_ == false){
            //we can send the current goal
            currentGoalSent_ = true;
            sendGoal(currentGoal_);
        }else{
            std::cout << "Service manager -> no new messages to send" << std::endl;
        }
    }
    

    void goalManagerTimer_callback(){
        //add logic for choosing robot ID for given tasks
        targetRobotID_ = 0;

        goalStruct newManagerGoal;
        bool availableObject = false;
        bool availableStorage = false;
        int objID = -1;
        
        switch (robots_.at(targetRobotID_).getStatus())
        {
        case WAITING:
            //send to input area box to pickup
            availableObject = storage_.generateGoalForObjectPickup(newManagerGoal, objID);
            if(!availableObject){
                std::cout << "Robot waiting, no objects to move" << std::endl;
                return; //nothing to do so return
            }else{
                //send goal
                std::cout << "Sending Goal - PICKUP object" << std::endl;
                robots_.at(targetRobotID_).setGoal(newManagerGoal);
                queueGoal(newManagerGoal);
                robots_.at(targetRobotID_).pickupDropoffObjectToggle(true, objID);
                robots_.at(targetRobotID_).setGoal(newManagerGoal);
            }
            break;

        case INPUT_TARGET:
            return; //skip entire function robot already heading to goal
            break;

        case OUTPUT_TARGET:
            return; //skip entire function robot already heading to goal
            break;

        case WAITING_WITH_OBJECT:
            availableStorage = storage_.getAvailableStoragePosition(newManagerGoal);
            if(!availableStorage){
                std::cout << "Robot waiting, no storage positions" << std::endl;
                return;
            }else{
                std::cout << "Sending Goal - STORE object" << std::endl;
                robots_.at(targetRobotID_).setGoal(newManagerGoal);
                queueGoal(newManagerGoal);
                robots_.at(targetRobotID_).pickupDropoffObjectToggle(false);
                robots_.at(targetRobotID_).setGoal(newManagerGoal);
            }
            break;
        
        default:
            break;
        }
    }

    void checkButtonClicked() {
        if (gui_->addItem == true) {
            std::cout << "[MASTER] regnise addItem command" << std::endl;
            // do something
            gui_->addItem = false;
        }

        else if (gui_->generateLiveReport == true) {
            std::cout << "[MASTER] regnise live report command" << std::endl;
            storage_.generatePDF(false);
            gui_->generateLiveReport = false;
        }

        else if (gui_->generateDailyReport == true) {
            std::cout << "[MASTER] regnise daily report command" << std::endl;
            //generate the data to pass
            dailyReportData data;
            data.numberOfRobots = robots_.size();
            for(size_t i = 0; i<robots_.size(); i++){
                data.robotDataVector.push_back(robots_.at(i).getRobotData());
            }

            data.storageDaily = storage_.generateStorageData();
            storage_.generatePDF(true, data);
            gui_->generateDailyReport = false;
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
 
    bool queueGoal(goalStruct newGoal){ //can only Queue one goal at a time
        if(newGoal == currentGoal_){
            std::cout << "new goal is EQUAL to current goal ignoring" << std::endl;
            return false;
        }else{
            currentGoalSent_ = false;
            currentGoal_ = newGoal;
        }
        return true;
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_NAV2POSE;
    
    rclcpp::TimerBase::SharedPtr goalAssignerTimer_;
    rclcpp::TimerBase::SharedPtr serviceManagerTimer_;
    rclcpp::TimerBase::SharedPtr checkButtonTimer;

    goalStruct currentGoal_;
    bool currentGoalSent_;
    
    std::vector<robot> robots_;
    
    int targetRobotID_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;

    //object storage class instance
    ObjectStorage storage_;

    double programStartXPos_;
    double programStartYPos_;
    double programStartYaw_;

    //gui stuff
    SimpleGUI* gui_;
    std::unique_ptr<QApplication> app_;
    rclcpp::TimerBase::SharedPtr timer_;

    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    GoalHandleNavigateToPose::SharedPtr current_goal_handle_;

    int testingCount_;
};


 
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MasterControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
