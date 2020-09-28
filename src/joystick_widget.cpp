/**********************************************************************************************
 * Joystick panel - Version 0.0.0
 * by Binit shah <bshah@ieee.org>
 *
 * Rviz2 panel to control differential drive robots using a joystick,
 * with configurable topic name, maximum velocities, and zero-ing behavior.
 *
 * This library is licensed under the MIT License
 **********************************************************************************************/

#include <stdio.h>
#include <math.h>
#include <QPainter>
#include <QMouseEvent>

#include "include/test_panel/joystick_widget.hpp"

namespace test_panel {

    JoystickWidget::JoystickWidget(QWidget* parent) :
        QWidget(parent), max_translational_velocity_(5.0), max_rotational_velocity_(2.0),
        translational_velocity_(0.0), rotational_velocity_(0.0) {

    }

    void JoystickWidget::paintEvent(QPaintEvent* event) {

    }

    void JoystickWidget::mouseMoveEvent(QMouseEvent* event) {

    }

    void JoystickWidget::mousePressEvent(QMouseEvent* event) {

    }

    void JoystickWidget::mouseReleaseEvent(QMouseEvent* event) {

    }

    void JoystickWidget::leaveEvent(QEvent* event) {

    }

}