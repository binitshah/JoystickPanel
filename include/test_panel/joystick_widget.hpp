/**********************************************************************************************
 * Joystick panel - Version 0.0.0
 * by Binit shah <bshah@ieee.org>
 *
 * Rviz2 panel to control differential drive robots using a joystick,
 * with configurable topic name, maximum velocities, and zero-ing behavior.
 *
 * This library is licensed under the MIT License
 **********************************************************************************************/

#ifndef JOYSTICK_WIDGET_H
#define JOYSTICK_WIDGET_H

#include <QWidget>

namespace test_panel {

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
            virtual void leaveEvent(QEvent* event);

            virtual QSize sizeHint() const { return QSize( 150, 150 ); }

            /*
             * TODO: A public method for others building on this panel
             * to set the vels which will be published in the Twist msg.
             */
            void getVelocities();

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
             * to set whether the joystick widget returns to
             * zero after being released.
             *
             * @param return_to_zero_ whether or not to go to zero
             */
            void setReturnToZero(bool return_to_zero);

        protected Q_SLOTS:
            void calculateVelocities();

        protected:
            virtual void processMouse(QPoint P);

            QPoint pos_;
            bool mouse_pressed_;
            bool return_to_zero_;
            QPoint last_error_;
            float max_translational_velocity_; // m/s
            float max_rotational_velocity_; // rad/s
            float translational_velocity_; // m/s
            float rotational_velocity_; // rad/s
    };
}

#endif // JOYSTICK_WIDGET_H
