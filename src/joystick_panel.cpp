/**********************************************************************************************
 * Joystick Panel - Version 0.0.0
 * by Binit shah <bshah@ieee.org>
 *
 * Rviz2 panel to control differential drive robots using a joystick,
 * with configurable topic name, maximum velocities, and zero-ing behavior.
 *
 * This library is licensed under the MIT License
 **********************************************************************************************/

#include "include/joystick_panel/joystick_panel.hpp"
#include "include/joystick_panel/joystick_widget.hpp"

#include <iostream>
#include <QLineEdit>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QString>


namespace joystick_panel {

    JoystickPanel::JoystickPanel(QWidget* parent) :
        Panel(parent), topic_("/cmd_vel") {
        // Setup ROS2 functionality
        node_ = rclcpp::Node::make_shared("joystick_panel");
        twist_publisher_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(topic_, 10);

        // Layout the panel
        joystick_widget_ = new JoystickWidget;
        auto max_vels = joystick_widget_->getMaxVelocities();
        bool is_return_to_zero = joystick_widget_->getReturnToZero();
        bool is_enabled = joystick_widget_->getEnabled();

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

        QHBoxLayout* checkboxes_layout = new QHBoxLayout;
        return_to_zero_gui_ = new QCheckBox("Return to Zero");
        return_to_zero_gui_->setChecked(is_return_to_zero);
        checkboxes_layout->addWidget(return_to_zero_gui_);
        // return_to_zero_gui_->setSizePolicy(QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum) );
        enabled_gui_ = new QCheckBox("Enabled");
        enabled_gui_->setChecked(is_enabled);
        checkboxes_layout->addWidget(enabled_gui_);
        // enabled_gui_->setSizePolicy(QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum) );

        QVBoxLayout* panel_layout = new QVBoxLayout;
        panel_layout->addWidget(new QLabel("Use mouse to move joystick. Enable/Disable with checkbox."));
        panel_layout->addWidget(joystick_widget_);
        panel_layout->addLayout(topic_layout);
        panel_layout->addLayout(max_vels_layout);
        panel_layout->addLayout(checkboxes_layout);
        setLayout(panel_layout);

        // Setup panel functionality
        QTimer* publish_timer = new QTimer(this);

        connect(topic_gui_, SIGNAL(editingFinished()), this, SLOT(updateTopic()));
        connect(max_translational_velocity_gui_, SIGNAL(editingFinished()), this, SLOT(updateMaxTranslationalVelocity()));
        connect(max_rotational_velocity_gui_, SIGNAL(editingFinished()), this, SLOT(updateMaxRotationalVelocity()));
        connect(return_to_zero_gui_, SIGNAL(toggled(bool)), this, SLOT(updateReturnToZero()));
        connect(enabled_gui_, SIGNAL(toggled(bool)), this, SLOT(updateEnabled()));
        connect(publish_timer, SIGNAL(timeout()), this, SLOT(publishVelocities()));

        publish_timer->start(100); // 0.1 seconds
    }

    void JoystickPanel::load(const rviz_common::Config& config) {
        Panel::load(config);
        QString topic;
        if (config.mapGetString("Topic", &topic)) {
            topic_gui_->setText(topic);
            updateTopic();
        }
        QString max_trans_velocity;
        if (config.mapGetString("max_translational_velocity", &max_trans_velocity)) {
            max_translational_velocity_gui_->setText(max_trans_velocity);
            updateMaxTranslationalVelocity();
        }
        QString max_rot_velocity;
        if (config.mapGetString("max_rotational_velocity", &max_rot_velocity)) {
            max_rotational_velocity_gui_->setText(max_rot_velocity);
            updateMaxRotationalVelocity();
        }
        QString is_return_to_zero;
        if (config.mapGetString("return_to_zero", &is_return_to_zero)) {
            return_to_zero_gui_->setChecked(is_return_to_zero == "false" ? false : true);
            updateReturnToZero();
        }
        QString is_enabled;
        if (config.mapGetString("enabled", &is_enabled)) {
            enabled_gui_->setChecked(is_enabled == "false" ? false : true);
            updateEnabled();
        }
    }

    void JoystickPanel::save(rviz_common::Config config) const {
        Panel::save(config);
        config.mapSetValue("Topic", QString::fromStdString(topic_));
        auto max_vels = joystick_widget_->getMaxVelocities();
        config.mapSetValue("max_translational_velocity", QString::number(std::get<0>(max_vels), 'f', 2));
        config.mapSetValue("max_rotational_velocity", QString::number(std::get<1>(max_vels), 'f', 2));
        bool is_return_to_zero = joystick_widget_->getReturnToZero();
        config.mapSetValue("return_to_zero", is_return_to_zero ? "true" : "false");
        bool is_enabled = joystick_widget_->getEnabled();
        config.mapSetValue("enabled", is_enabled ? "true" : "false");
    }

    void JoystickPanel::publishVelocities() {
        if (joystick_widget_->getEnabled()) {
            geometry_msgs::msg::TwistStamped msg;
            msg.header.stamp = node_->get_clock()->now();
            auto vels = joystick_widget_->getVelocities();
            msg.twist.linear.x = std::get<0>(vels);
            msg.twist.angular.z = std::get<1>(vels);
            twist_publisher_->publish(msg);
        }
    }

    void JoystickPanel::updateTopic() {
        if (!setTopic(topic_gui_->text().toStdString())) {
            topic_gui_->setText(QString::fromStdString(topic_));
        }
    }

    void JoystickPanel::updateMaxTranslationalVelocity() {
        bool ok;
        float max_trans_vel = max_translational_velocity_gui_->text().toFloat(&ok);
        if (!ok || !joystick_widget_->setMaxTransVelocities(max_trans_vel)) {
            auto max_vels = joystick_widget_->getMaxVelocities();
            max_translational_velocity_gui_->setText(QString::number(std::get<0>(max_vels), 'f', 2));
        }
    }

    void JoystickPanel::updateMaxRotationalVelocity() {
        bool ok;
        float max_rot_vel = max_rotational_velocity_gui_->text().toFloat(&ok);
        if (!ok || !joystick_widget_->setMaxRotVelocities(max_rot_vel)) {
            auto max_vels = joystick_widget_->getMaxVelocities();
            max_rotational_velocity_gui_->setText(QString::number(std::get<1>(max_vels), 'f', 2));
        }
    }

    void JoystickPanel::updateReturnToZero() {
        joystick_widget_->setReturnToZero(return_to_zero_gui_->isChecked());
    }

    void JoystickPanel::updateEnabled() {
        joystick_widget_->setEnabled(enabled_gui_->isChecked());
    }

    bool JoystickPanel::setTopic(const std::string& topic) {
        if (topic == "" || isdigit(topic[0]) || topic[topic.length()-1] == '/') {
            return false;
        }

        topic_ = topic;
        twist_publisher_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(topic_, 10);
        return true;
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(joystick_panel::JoystickPanel, rviz_common::Panel)
