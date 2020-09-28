/**********************************************************************************************
 * Joystick panel - Version 0.0.0
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
#include <geometry_msgs/msg/twist.hpp>

class QLineEdit;
class QCheckBox;

namespace test_panel {

    class JoystickWidget;

    /**
     * \class JoystickPanel
     * 
     */
    class JoystickPanel : public rviz_common::Panel {
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
             */
            void setTopic(const std::string& topic);

            /*
             * A public method for others building on this panel
             * to set the maximum vels that the joystick widget can
             * reach set for publishing in the Twist msg.
             *
             * @param max_translational_velocity maximum linear vel in m/s
             * @param max_rotational_velocity    maximum angular vel in rad/s
             */
            void setMaxVelocities(float max_translational_velocity, float max_rotational_velocity);

            /*
             * A public method for others building on this panel
             * to set the vels which will be published in the Twist msg.
             *
             * @param translational_velocity linear vel in m/s
             * @param rotational_velocity    angular vel in rad/s
             */
            void setVelocities(float translational_velocity, float rotational_velocity);

            /*
             * A public method for others building on this panel
             * to set whether the joystick widget returns to
             * zero after being released.
             *
             * @param return_to_zero whether or not to go to zero
             */
            void setReturnToZero(bool return_to_zero);

        protected Q_SLOTS:

            /*
             * Uses the ROS2 publisher to publish a
             * geometry_msgs/Twist.msg.
             */
            void publishVelocities();

            /*
             * The GUI's callback for updating the topic name at
             * which to publish the Twist msg.
             */
            void updateTopic();

            /*
             * The joystick GUI's callback for updating the vels
             * which will be published in the Twist msg.
             *
             * @param translational_velocity linear vel in m/s
             * @param rotational_velocity    angular vel in rad/s
             */
            void updateVelocities(float translational_velocity, float rotational_velocity);

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

        protected:
            // Panel vars
            JoystickWidget* joystick_widget_;
            QLineEdit* topic_gui_;
            QLineEdit* max_translational_velocity_gui_;
            QLineEdit* max_rotational_velocity_gui_;
            QCheckBox* return_to_zero_gui_;
            std::string topic_;
            float max_translational_velocity_;
            float max_rotational_velocity_;
            float translational_velocity_;
            float rotational_velocity_;
            bool return_to_zero_;

            // ROS2 vars
            std::shared_ptr<rclcpp::Node> node_;
            std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> twist_publisher_;
    };
}

#endif // JOYSTICK_PANEL_H
