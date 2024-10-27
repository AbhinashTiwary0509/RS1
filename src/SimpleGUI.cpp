#include "SimpleGUI.hpp"

SimpleGUI::SimpleGUI(rclcpp::Node::SharedPtr node, QWidget *parent)
    : QMainWindow(parent), node_(node)
{
    // Set window properties
    setWindowTitle("ROS2 Control GUI");
    setMinimumSize(400, 300);  // Set minimum window size

    // Create and set the central widget
    centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    // Create the layout
    layout = new QVBoxLayout(centralWidget);
    layout->setSpacing(10);  // Add some space between buttons
    layout->setContentsMargins(10, 10, 10, 10);  // Add margins around the edges
    
    // Create buttons with minimum size
    button1 = new QPushButton("Add Item", centralWidget);
    button2 = new QPushButton("Generate Live Report", centralWidget);
    button3 = new QPushButton("Generate Daily Report", centralWidget);
    button4 = new QPushButton("Button 4", centralWidget);

    // Set minimum button size
    QSize buttonSize(150, 40);
    button1->setMinimumSize(buttonSize);
    button2->setMinimumSize(buttonSize);
    button3->setMinimumSize(buttonSize);
    button4->setMinimumSize(buttonSize);
    
    // Add buttons to layout
    layout->addWidget(button1);
    layout->addWidget(button2);
    layout->addWidget(button3);
    layout->addWidget(button4);

    // Add stretch at the end to push buttons to the top
    layout->addStretch();
    
    // Connect buttons to slots
    connect(button1, &QPushButton::clicked, this, &SimpleGUI::onButton1Clicked);
    connect(button2, &QPushButton::clicked, this, &SimpleGUI::onButton2Clicked);
    connect(button3, &QPushButton::clicked, this, &SimpleGUI::onButton3Clicked);
    connect(button4, &QPushButton::clicked, this, &SimpleGUI::onButton4Clicked);
}

void SimpleGUI::onButton1Clicked()
{
    RCLCPP_INFO(node_->get_logger(), "Add Item to be stored!");
    addItem = true;
    // Add your function call here
}

void SimpleGUI::onButton2Clicked()
{
    RCLCPP_INFO(node_->get_logger(), "Generating Live Report!");
    generateLiveReport = true;
}

void SimpleGUI::onButton3Clicked()
{
    RCLCPP_INFO(node_->get_logger(), "Generating Daily Report!");
    generateDailyReport = true;
    // Add your function call here
}

void SimpleGUI::onButton4Clicked()
{
    RCLCPP_INFO(node_->get_logger(), "Button 4 clicked!");
    // Add your function call here
}
