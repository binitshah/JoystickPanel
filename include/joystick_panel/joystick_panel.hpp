/**********************************************************************************************
 * Joystick Panel - Version 0.0.0
 * by Binit shah <bshah@ieee.org>
 *
 * Rviz2 panel to control differential drive robots using a joystick,
 * with configurable topic name, maximum velocities, and zero-ing behavior.
 *
 * This library is licensed under the MIT License
 **********************************************************************************************/

#ifndef JOYSTICK_PANEL_H
#define JOYSTICK_PANEL_H

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class QLineEdit;
class QCheckBox;

namespace joystick_panel {

    class JoystickWidget;

    /*
     * Defines the panel.
     */
    class JoystickPanel: public rviz_common::Panel {
        Q_OBJECT

        public:
            JoystickPanel(QWidget* parent=nullptr);
            virtual ~JoystickPanel() = default;

            /*
             * Loads an Rviz config file and populates panel config.
             *
             * @param config the file from which to read panel config
             */
            virtual void load(const rviz_common::Config& config);

            /*
             * Save an Rviz config file with panel config.
             *
             * @param config the file to which to write panel config
             */
            virtual void save(rviz_common::Config config) const;

        public Q_SLOTS:

            /*
             * A public method for others building on this panel
             * to set the topic name at which to publish the Twist msg.
             *
             * @param topic a string representing the topic name
             * @return bool whether the given topic param is valid
             */
            bool setTopic(const std::string& topic);

        protected Q_SLOTS:

            /*
             * Uses the ROS2 publisher to publish a
             * geometry_msgs/TwistStamped.msg.
             */
            void publishVelocities();

            /*
             * The GUI's callback for updating the topic name at
             * which to publish the TwistStamped msg.
             */
            void updateTopic();

            /*
             * The GUI's callback for updating the max rotational
             * velocity that the joystick widget can set.
             */
            void updateMaxTranslationalVelocity();

            /*
             * The GUI's callback for updating the max translational
             * velocity that the joystick widget can set.
             */
            void updateMaxRotationalVelocity();

            /*
             * The GUI's callback for updating the return to
             * zero behavior.
             */
            void updateReturnToZero();

            /*
             * The GUI's callback for updating the enabled
             * checkbox.
             */
            void updateEnabled();

        protected:
            // Panel vars
            JoystickWidget* joystick_widget_;
            QLineEdit* topic_gui_;
            QLineEdit* max_translational_velocity_gui_;
            QLineEdit* max_rotational_velocity_gui_;
            QCheckBox* return_to_zero_gui_;
            QCheckBox* enabled_gui_;
            std::string topic_;

            // ROS2 vars
            std::shared_ptr<rclcpp::Node> node_;
            std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> twist_publisher_;
    };
}

#endif // JOYSTICK_PANEL_H
