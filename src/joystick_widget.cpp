/**********************************************************************************************
 * Joystick Widget - Version 0.0.0
 * by Binit shah <bshah@ieee.org>
 *
 * Rviz2 TODO.
 *
 * This library is licensed under the MIT License
 **********************************************************************************************/


#include "include/joystick_panel/joystick_widget.hpp"

#include <iostream>
#include <math.h>
#include <QPainter>
#include <QPainterPath>
#include <QMouseEvent>
#include <QTimer>


namespace joystick_panel {

    JoystickWidget::JoystickWidget(QWidget* parent) :
        QWidget(parent), max_translational_velocity_(5.0), max_rotational_velocity_(2.0),
        translational_velocity_(0.0), rotational_velocity_(0.0),
        mouse_pressed_(false), return_to_zero_(true), enabled_(false) {
        QTimer* calculation_timer = new QTimer(this);
        connect(calculation_timer, SIGNAL(timeout()), this, SLOT(calculateVelocities()));
        calculation_timer->start(10); // 0.01 seconds
    }

    void JoystickWidget::calculateVelocities() {
        // Calculate widget boundary, TODO: do only once
        int boundary_width = width();
        int boundary_height = height();
        int boundary_size = std::min(boundary_width, boundary_height) - 1;
        int boundary_hpad = (boundary_width - boundary_size) / 2;
        int boundary_vpad = (boundary_height - boundary_size) / 2;
        QRect boundary(boundary_hpad, boundary_vpad, boundary_size, boundary_size);
        int inner_radius = boundary_size / 6;
        int outer_radius = boundary_size / 2 - inner_radius / 2;

        if (!mouse_pressed_ && return_to_zero_) {
            int currx_error = pos_.x() - boundary.center().x();
            int curry_error = pos_.y() - boundary.center().y();
            if ((std::abs(currx_error) + std::abs(curry_error)) > 8) {
                if (last_error_.x() == 0.0 && last_error_.y() == 0.0) {
                    last_error_ = QPoint(currx_error, curry_error);
                }
                QPoint pd = QPoint(0.2 * currx_error + 0.2 * (currx_error - last_error_.x()),
                                   0.2 * curry_error + 0.2 * (curry_error - last_error_.y()));
                pos_ -= pd;
                last_error_ = QPoint(currx_error, curry_error);
            } else {
                pos_ = boundary.center();
            }

            update();
        }

        rotational_velocity_ = (-(pos_.x() - boundary.center().x()) / (float)outer_radius) * max_rotational_velocity_;
        translational_velocity_ = (-(pos_.y() - boundary.center().y()) / (float)outer_radius) * max_translational_velocity_;
    }

    void JoystickWidget::paintEvent(QPaintEvent* event __attribute__((unused))) {
        QPainter painter(this);

        // Calculate widget boundary
        int boundary_width = width();
        int boundary_height = height();
        int boundary_size = std::min(boundary_width, boundary_height) - 1;
        int boundary_hpad = (boundary_width - boundary_size) / 2;
        int boundary_vpad = (boundary_height - boundary_size) / 2;
        QRect boundary(boundary_hpad, boundary_vpad, boundary_size, boundary_size);

        // Draw the outer ellipse of the joystick
        int inner_radius = boundary_size / 6;
        int outer_radius = boundary_size / 2 - inner_radius / 2;
        painter.setPen(QColor(245, 245, 245));
        painter.setBrush(QColor(80, 80, 80));
        painter.drawEllipse(boundary.center(), outer_radius, outer_radius);

        // Draw the four arrows on the outer ellipse
        int arrow_width = boundary_size / 16;
        int arrow_height = boundary_size / 16;
        QPoint top_arrow_origin = QPoint(boundary.center().x() - arrow_width / 2, boundary.center().y() - outer_radius * 0.75);
        QPainterPath top_arrow;
        top_arrow.moveTo(top_arrow_origin.x(), top_arrow_origin.y());
        top_arrow.lineTo(top_arrow_origin.x() + arrow_width, top_arrow_origin.y());
        top_arrow.lineTo(top_arrow_origin.x() + arrow_width / 2, top_arrow_origin.y() - arrow_height);
        top_arrow.lineTo(top_arrow_origin.x(), top_arrow_origin.y());
        QPoint bottom_arrow_origin = QPoint(boundary.center().x() - arrow_width / 2, boundary.center().y() + outer_radius * 0.75);
        QPainterPath bottom_arrow;
        bottom_arrow.moveTo(bottom_arrow_origin.x(), bottom_arrow_origin.y());
        bottom_arrow.lineTo(bottom_arrow_origin.x() + arrow_width, bottom_arrow_origin.y());
        bottom_arrow.lineTo(bottom_arrow_origin.x() + arrow_width / 2, bottom_arrow_origin.y() + arrow_height);
        bottom_arrow.lineTo(bottom_arrow_origin.x(), bottom_arrow_origin.y());
        QPoint right_arrow_origin = QPoint(boundary.center().x() + outer_radius * 0.75, boundary.center().y() - arrow_width / 2);
        QPainterPath right_arrow;
        right_arrow.moveTo(right_arrow_origin.x(), right_arrow_origin.y());
        right_arrow.lineTo(right_arrow_origin.x(), right_arrow_origin.y() + arrow_width);
        right_arrow.lineTo(right_arrow_origin.x() + arrow_height, right_arrow_origin.y() + arrow_width / 2);
        right_arrow.lineTo(right_arrow_origin.x(), right_arrow_origin.y());
        QPoint left_arrow_origin = QPoint(boundary.center().x() - outer_radius * 0.75, boundary.center().y() - arrow_width / 2);
        QPainterPath left_arrow;
        left_arrow.moveTo(left_arrow_origin.x(), left_arrow_origin.y());
        left_arrow.lineTo(left_arrow_origin.x(), left_arrow_origin.y() + arrow_width);
        left_arrow.lineTo(left_arrow_origin.x() - arrow_height, left_arrow_origin.y() + arrow_width / 2);
        left_arrow.lineTo(left_arrow_origin.x(), left_arrow_origin.y());
        painter.setPen(Qt::NoPen);
        painter.fillPath(top_arrow, QBrush(QColor(245, 245, 245)));
        painter.fillPath(bottom_arrow, QBrush(QColor(245, 245, 245)));
        painter.fillPath(right_arrow, QBrush(QColor(245, 245, 245)));
        painter.fillPath(left_arrow, QBrush(QColor(245, 245, 245)));

        // Draw the inner ellipse of the joystick
        if (pos_.x() == 0.0 && pos_.y() == 0.0) { // pos_ initalizes at zeros
            pos_ = boundary.center();
        }
        painter.setPen(QColor(100, 100, 100));
        painter.setBrush(QColor(245, 245, 245));
        painter.drawEllipse(pos_, inner_radius, inner_radius);
    }

    void JoystickWidget::mouseMoveEvent(QMouseEvent* event) {
        processMouse(QPoint(event->x(), event->y()));
        update();
    }

    void JoystickWidget::mousePressEvent(QMouseEvent* event) {
        mouse_pressed_ = true;
        processMouse(QPoint(event->x(), event->y()));
        update();
    }

    void JoystickWidget::mouseReleaseEvent(QMouseEvent* event __attribute__((unused))) {
        mouse_pressed_ = false;
    }

    void JoystickWidget::processMouse(QPoint P) {
        int boundary_width = width();
        int boundary_height = height();
        int boundary_size = std::min(boundary_width, boundary_height) - 1;
        int boundary_hpad = (boundary_width - boundary_size) / 2;
        int boundary_vpad = (boundary_height - boundary_size) / 2;
        int inner_radius = boundary_size / 6;
        int outer_radius = boundary_size / 2 - inner_radius / 2;
        QRect boundary(boundary_hpad, boundary_vpad, boundary_size, boundary_size);

        QPoint C = boundary.center();
        if (std::sqrt(std::pow(P.x() - C.x(), 2) + std::pow(P.y() - C.y(), 2)) > outer_radius) {
            QPoint V = P - C;
            double magV = std::sqrt(std::pow(V.x(), 2) + std::pow(V.y(), 2));
            int aX = C.x() + V.x() / magV * outer_radius;
            int aY = C.y() + V.y() / magV * outer_radius;
            pos_ = QPoint(aX, aY);
        } else {
            pos_ = P;
        }
    }

    std::tuple<float, float> JoystickWidget::getVelocities() {
        return std::make_tuple(translational_velocity_, rotational_velocity_);
    }

    std::tuple<float, float> JoystickWidget::getMaxVelocities() {
        return std::make_tuple(max_translational_velocity_, max_rotational_velocity_);
    }

    bool JoystickWidget::setMaxTransVelocities(float max_translational_velocity) {
        if (max_translational_velocity < 0.0) {
            return false;
        }

        max_translational_velocity_ = max_translational_velocity;
        return true;
    }

    bool JoystickWidget::setMaxRotVelocities(float max_rotational_velocity) {
        if (max_rotational_velocity < 0.0) {
            return false;
        }

        max_rotational_velocity_ = max_rotational_velocity;
        return true;
    }

    bool JoystickWidget::getReturnToZero() {
        return return_to_zero_;
    }

    void JoystickWidget::setReturnToZero(bool return_to_zero) {
        return_to_zero_ = return_to_zero;
    }

    bool JoystickWidget::getEnabled() {
        return enabled_;
    }

    void JoystickWidget::setEnabled(bool enabled) {
        enabled_ = enabled;
    }
}
