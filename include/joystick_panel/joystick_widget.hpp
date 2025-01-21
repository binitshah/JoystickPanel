/**********************************************************************************************
 * Joystick Widget - Version 0.0.0
 * by Binit shah <bshah@ieee.org>
 *
 * Rviz2 TODO.
 *
 * This library is licensed under the MIT License
 **********************************************************************************************/

#ifndef JOYSTICK_WIDGET_H
#define JOYSTICK_WIDGET_H

#include <QWidget>

namespace joystick_panel {

    /*
     * Defines the widget.
     */
    class JoystickWidget: public QWidget {
        Q_OBJECT

        public:
            JoystickWidget(QWidget* parent=nullptr);
            virtual ~JoystickWidget() = default;

            virtual void paintEvent(QPaintEvent* event);
            virtual void mouseMoveEvent(QMouseEvent* event);
            virtual void mousePressEvent(QMouseEvent* event);
            virtual void mouseReleaseEvent(QMouseEvent* event);
            virtual QSize sizeHint() const { return QSize( 150, 150 ); }

            /*
             * A public method for others building on this panel
             * to set the vels which will be published in the Twist msg.
             */
            std::tuple<float, float> getVelocities();

            /*
             * A public method for others building on this panel
             * to set the vels which will be published in the Twist msg.
             */
            std::tuple<float, float> getMaxVelocities();

            /*
             * A public method for others building on this panel
             * to set whether the joystick widget returns to
             * zero after being released.
             */
            bool getReturnToZero();

            /*
             * A public method for others building on this panel
             * to set whether the joystick widget is enabled.
             */
            bool getEnabled();

            /*
             * A public method for others building on this panel
             * to set whether the topic is publishing stamped twist messages
             */
            bool getStamped();

            /*
             * A public method for others building on this panel
             * to set the maximum vels that the joystick widget can
             * reach set for publishing in the Twist msg.
             *
             * @param max_translational_velocity maximum linear vel in m/s
             * @return bool whether param is valid
             */
            bool setMaxTransVelocities(float max_translational_velocity);

            /*
             * A public method for others building on this panel
             * to set the maximum vels that the joystick widget can
             * reach set for publishing in the Twist msg.
             *
             * @param max_rotational_velocity    maximum angular vel in rad/s
             * @return bool whether param is valid
             */
            bool setMaxRotVelocities(float max_rotational_velocity);

            /*
             * A public method for others building on this panel
             * to set whether the joystick widget returns to
             * zero after being released.
             *
             * @param return_to_zero whether or not to go to zero
             */
            void setReturnToZero(bool return_to_zero);

            /*
             * A public method for others building on this panel
             * to set whether the joystick widget is enabled.
             *
             * @param enabled whether or not the widget is enabled
             */
            void setEnabled(bool enabled);

            /*
             * A public method for others building on this panel
             * to set whether the joystick widget publishes TwistStamped messages
             *
             * @param enabled whether or not the widget publishes stamped messages
             */
            void setStamped(bool enabled);

        protected Q_SLOTS:
            void calculateVelocities();

        protected:
            virtual void processMouse(QPoint P);

            QPoint pos_;
            bool mouse_pressed_;
            bool return_to_zero_;
            bool enabled_;
            bool stamped_;
            QPoint last_error_;
            float max_translational_velocity_; // m/s
            float max_rotational_velocity_; // rad/s
            float translational_velocity_; // m/s
            float rotational_velocity_; // rad/s
    };
}

#endif // JOYSTICK_WIDGET_H
