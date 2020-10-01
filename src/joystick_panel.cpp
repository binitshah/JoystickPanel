/**********************************************************************************************
 * Joystick panel - Version 0.0.0
 * by Binit shah <bshah@ieee.org>
 *
 * Rviz2 panel to control differential drive robots using a joystick,
 * with configurable topic name, maximum velocities, and zero-ing behavior.
 *
 * This library is licensed under the MIT License
 **********************************************************************************************/

#include "include/test_panel/joystick_panel.hpp"
#include "include/test_panel/joystick_widget.hpp"

#include <iostream>
#include <QLineEdit>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QString>


namespace test_panel {

    JoystickPanel::JoystickPanel(QWidget* parent) :
        Panel(parent), topic_("/cmd_vel") {
        // Setup ROS2 functionality
        node_ = rclcpp::Node::make_shared("joystick_panel");
        twist_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(topic_, 10);

        // Layout the panel
        joystick_widget_ = new JoystickWidget;
        auto max_vels = joystick_widget_->getMaxVelocities();
        bool is_return_to_zero = joystick_widget_->getReturnToZero();

        QHBoxLayout* topic_layout = new QHBoxLayout;
        topic_layout->addWidget(new QLabel("Topic: "));
        topic_gui_ = new QLineEdit;
        topic_gui_->setText(QString::fromStdString(topic_));
        topic_layout->addWidget(topic_gui_);

        QHBoxLayout* max_vels_layout = new QHBoxLayout;
        max_vels_layout->addWidget(new QLabel("Max Lin (m/s): "));
        max_translational_velocity_gui_ = new QLineEdit;
        max_translational_velocity_gui_->setText(QString::number(std::get<0>(max_vels), 'f', 2));
        max_vels_layout->addWidget(max_translational_velocity_gui_);
        max_vels_layout->addWidget(new QLabel("Max Ang (rad/s): "));
        max_rotational_velocity_gui_ = new QLineEdit;
        max_rotational_velocity_gui_->setText(QString::number(std::get<1>(max_vels), 'f', 2));
        max_vels_layout->addWidget(max_rotational_velocity_gui_);

        return_to_zero_gui_ = new QCheckBox("Return to Zero");
        return_to_zero_gui_->setChecked(is_return_to_zero);
        // return_to_zero_gui_->setSizePolicy(QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum) );

        QVBoxLayout* panel_layout = new QVBoxLayout;
        panel_layout->addWidget(new QLabel("Use arrow keys or mouse. Press 'space' to stop."));
        panel_layout->addWidget(joystick_widget_);
        panel_layout->addLayout(topic_layout);
        panel_layout->addLayout(max_vels_layout);
        panel_layout->addWidget(return_to_zero_gui_);
        setLayout(panel_layout);

        // Setup panel functionality
        QTimer* publish_timer = new QTimer(this);

        connect(topic_gui_, SIGNAL(editingFinished()), this, SLOT(updateTopic()));
        connect(max_translational_velocity_gui_, SIGNAL(editingFinished()), this, SLOT(updateMaxTranslationalVelocity()));
        connect(max_rotational_velocity_gui_, SIGNAL(editingFinished()), this, SLOT(updateMaxRotationalVelocity()));
        connect(return_to_zero_gui_, SIGNAL(toggled(bool)), this, SLOT(updateReturnToZero()));
        connect(publish_timer, SIGNAL(timeout()), this, SLOT(publishVelocities()));

        publish_timer->start(100); // 0.1 seconds
    }

    void JoystickPanel::load(const rviz_common::Config& config) {
        Panel::load(config);
        QString topic;
        if (config.mapGetString("Topic", &topic)) {
            if (!topic.isEmpty()) {
                topic_gui_->setText(topic);
                setTopic(topic.toStdString());
            }
        }
    }

    void JoystickPanel::save(rviz_common::Config config) const {
        Panel::save(config);
        config.mapSetValue("Topic", QString::fromStdString(topic_));
        // config.mapSetValue("max_translational_velocity", max_translational_velocity_);
        // config.mapSetValue("max_rotational_velocity", max_rotational_velocity_);
        // config.mapSetValue("return_to_zero", return_to_zero_);
        // TODO
    }

    void JoystickPanel::publishVelocities() {
        geometry_msgs::msg::Twist msg;
        auto vels = joystick_widget_->getVelocities();
        msg.linear.x = std::get<0>(vels);
        msg.angular.z = std::get<1>(vels);
        twist_publisher_->publish(msg);
    }

    void JoystickPanel::updateTopic() {
        setTopic(topic_gui_->text().toStdString());
    }

    void JoystickPanel::updateMaxTranslationalVelocity() {
        // TODO
        // joystick_widget_.setMaxVelocities()
    }

    void JoystickPanel::updateMaxRotationalVelocity() {
        // TODO: how to read float
        // joystick_widget_.setMaxVelocities()
    }

    void JoystickPanel::updateReturnToZero() {
        std::cout << return_to_zero_gui_->isChecked() << std::endl;
        joystick_widget_->setReturnToZero(return_to_zero_gui_->isChecked());
    }

    void JoystickPanel::setTopic(const std::string& topic) {
        if (topic != "") {
            topic_ = topic;
            twist_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(topic_, 10);
        }
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(test_panel::JoystickPanel, rviz_common::Panel)
