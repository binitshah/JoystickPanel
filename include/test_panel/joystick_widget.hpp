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

        protected:
            float max_translational_velocity_; // m/s
            float max_rotational_velocity_; // rad/s
            float translational_velocity_; // m/s
            float rotational_velocity_; // rad/s
    };
}

#endif // JOYSTICK_WIDGET_H