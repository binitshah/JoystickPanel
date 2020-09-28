/**********************************************************************************************
 * Joystick panel - Version 0.0.0
 * by Binit shah <bshah@ieee.org>
 *
 * Rviz2 panel to control differential drive robots using a joystick,
 * with configurable topic name, maximum velocities, and zero-ing behavior.
 *
 * This library is licensed under the MIT License
 **********************************************************************************************/

#include <QLineEdit>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QString>


#include <geometry_msgs/msg/twist.hpp>

#include "test_panel/joystick_panel.hpp"

namespace test_panel {

    JoystickPanel::JoystickPanel(QWidget* parent) :
        Panel(parent), translational_velocity_(0.0), rotational_velocity_(0.0) {
        // Layout the panel
        QHBoxLayout* topic_layout = new QHBoxLayout;
        topic_layout->addWidget(new QLabel("Topic: "));
        topic_editor_ = new QLineEdit;
        topic_layout->addWidget(topic_editor_);

        QHBoxLayout* max_translational_vel_layout = new QHBoxLayout;
        max_translational_vel_layout->addWidget(new QLabel("Max translational velocity: "));
        max_translational_velocity_editor_ = new QLineEdit;
        max_translational_vel_layout->addWidget(max_translational_velocity_editor_);

        QHBoxLayout* max_rotational_vel_layout = new QHBoxLayout;
        max_rotational_vel_layout->addWidget(new QLabel("Max rotational velocity: "));
        max_rotational_velocity_editor_ = new QLineEdit;
        max_rotational_vel_layout->addWidget(max_rotational_velocity_editor_);

        QHBoxLayout* return_to_zero_layout = new QHBoxLayout;
        return_to_zero_layout->addWidget(new QLabel("Return to Zero"));

        QVBoxLayout* panel_layout = new QVBoxLayout;
        panel_layout->addWidget(new QLabel("Use arrow keys. Press 'space' to stop."));
        panel_layout->addLayout(topic_layout);
        panel_layout->addLayout(max_translational_vel_layout);
        panel_layout->addLayout(max_rotational_vel_layout);
        panel_layout->addLayout(return_to_zero_layout);
        setLayout(panel_layout);

        // Setup panel functionality
        QTimer* publish_timer = new QTimer(this);

        connect(topic_editor_, SIGNAL(editingFinished()), this, SLOT(updateTopic()));
        connect(max_translational_velocity_editor_, SIGNAL(editingFinished()), this, SLOT(updateMaxTranslationalVelocity()));
        connect(max_rotational_velocity_editor_, SIGNAL(editingFinished()), this, SLOT(updateMaxRotationalVelocity()));
        connect(publish_timer, SIGNAL(timeout()), this, SLOT(publishVelocities()));

        publish_timer->start(100); // 0.1 seconds
    }

    void JoystickPanel::load(const rviz_common::Config& config) {
        Panel::load(config);
        QString topic;
        if (config.mapGetString("Topic", &topic)) {
            topic_editor_->setText(topic);
            setTopic(topic);
        }
        // TODO: implement default
    }

    void JoystickPanel::save(rviz_common::Config config) const {
        Panel::save(config);
        config.mapSetValue("Topic", topic_);
        // config.mapSetValue("max_translational_velocity", max_translational_velocity_);
        // config.mapSetValue("max_rotational_velocity", max_rotational_velocity_);
        // config.mapSetValue("return_to_zero", return_to_zero_);
        // TODO
    }

    void JoystickPanel::publishVelocities() {
        // TODO
    }

    void JoystickPanel::updateTopic() {
        // TODO
    }

    void JoystickPanel::updateVelocities(float translational_velocity, float rotational_velocity) {
        translational_velocity_ = translational_velocity;
        rotational_velocity_ = rotational_velocity;
    }

    void JoystickPanel::updateMaxTranslationalVelocity() {
        // TODO
    }

    void JoystickPanel::updateMaxRotationalVelocity() {
        // TODO
    }

    void JoystickPanel::updateReturnToZero() {
        // TODO
    }

    void JoystickPanel::setTopic(const QString& topic) {
        if (topic != "") {
            topic_ = topic; // TODO: handle changing publisher to new topic
        }
    }

    void JoystickPanel::setMaxVelocities(float max_translational_velocity, float max_rotational_velocity) {
        max_translational_velocity_ = max_translational_velocity;
        max_rotational_velocity_ = max_rotational_velocity;
    }

    void JoystickPanel::setVelocities(float translational_velocity, float rotational_velocity) {
        translational_velocity_ = translational_velocity;
        rotational_velocity_ = rotational_velocity;
    }

    void JoystickPanel::setReturnToZero(bool return_to_zero) {
        // TODO
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(test_panel::JoystickPanel, rviz_common::Panel)
