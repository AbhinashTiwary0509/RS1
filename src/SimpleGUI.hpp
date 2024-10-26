#ifndef SIMPLE_GUI_HPP
#define SIMPLE_GUI_HPP

#include <QMainWindow>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include "rclcpp/rclcpp.hpp"

class SimpleGUI : public QMainWindow
{
    Q_OBJECT

public:
    SimpleGUI(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
    ~SimpleGUI() = default;

private slots:
    void onButton1Clicked();
    void onButton2Clicked();
    void onButton3Clicked();
    void onButton4Clicked();

private:
    rclcpp::Node::SharedPtr node_;
    QPushButton *button1;
    QPushButton *button2;
    QPushButton *button3;
    QPushButton *button4;
    QWidget *centralWidget;
    QVBoxLayout *layout;
};

#endif