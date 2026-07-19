// FourBar.hpp

#pragma once
#include "EZ-Template/PID.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

class FourBar {
 public:
  // motor_group: both four-bar motors (mirrored, wired/reversed so positive voltage = lift up)
  // rotation_sensor: mounted on the four-bar's output shaft for position feedback
  // position_pid_constants: outer loop, converts height error -> target velocity
  // velocity_pid_constants: inner loop, converts velocity error -> motor voltage
  // stopping_threshold: how close (in inches) counts as "settled" for wait_until_settled()
  // near_threshold: how close (in inches) before switching from max-speed to cascaded PID
  // degrees_per_inch: conversion factor from rotation sensor degrees to linear height (inches)
  FourBar(pros::MotorGroup* motor_group, pros::Rotation* rotation_sensor,
          ez::PID::Constants position_pid_constants, ez::PID::Constants velocity_pid_constants,
          double stopping_threshold, double near_threshold, double degrees_per_inch)
      : motor_group(motor_group),
        rotation_sensor(rotation_sensor),
        position_pid_constants(position_pid_constants),
        velocity_pid_constants(velocity_pid_constants),
        position_pid(position_pid_constants.kp, position_pid_constants.ki, position_pid_constants.kd, position_pid_constants.start_i),
        velocity_pid(velocity_pid_constants.kp, velocity_pid_constants.ki, velocity_pid_constants.kd, velocity_pid_constants.start_i),
        stopping_threshold(stopping_threshold),
        near_threshold(near_threshold),
        degrees_per_inch(degrees_per_inch),
        task([this] { this->task_loop(); }) {}

  // Set a new target height, in inches
  void set_height(double inches) {
    target_height = inches;
    position_pid.target_set(target_height);
  }

  // Get the current target height, in inches
  double get_target_height() {
    return target_height;
  }

  // Reset both PID controllers (call before a fresh movement, or after re-enabling the mechanism)
  void reset_controllers() {
    position_pid.variables_reset();
    velocity_pid.variables_reset();
    position_pid.target_set(target_height);
  }

  // Returns true if we're within stopping_threshold (inches) of target
  bool is_moving() {
    double error = target_height - get_height();
    return fabs(error) > stopping_threshold;
  }

  // Blocks until is_moving() becomes false
  void wait_until_settled() {
    while (is_moving()) {
      pros::delay(10);
    }
  }

 private:
  // Converts rotation sensor reading (centidegrees) into height (inches)
  double get_height() {
    double degrees = rotation_sensor->get_position() / 100.0;
    return degrees / degrees_per_inch;
  }

  // Runs forever in the background task
  void task_loop() {
    while (true) {
      double current_height = get_height();
      double error = target_height - current_height;

      if (fabs(error) > near_threshold) {
        // Far from target: full voltage in the direction of travel
        double direction = (error > 0) ? 1.0 : -1.0;
        motor_group->move_voltage(direction * 12000);
      } else {
        // Near target: cascaded PID takes over for accurate settling
        // Outer loop: position error -> target velocity (deg/sec)
        double target_velocity = position_pid.compute(current_height);

        // Inner loop: velocity error -> voltage
        double current_velocity = motor_group->get_actual_velocity();  // RPM, averaged across group
        velocity_pid.target_set(target_velocity);
        double output = velocity_pid.compute(current_velocity);

        motor_group->move_voltage(output * (12000.0 / 100.0));
      }

      pros::delay(10);
    }
  }

  pros::MotorGroup* motor_group;
  pros::Rotation* rotation_sensor;

  ez::PID::Constants position_pid_constants;  // outer loop config
  ez::PID::Constants velocity_pid_constants;  // inner loop config
  ez::PID position_pid;                       // outer loop: height error -> target velocity
  ez::PID velocity_pid;                       // inner loop: velocity error -> voltage

  double stopping_threshold;  // inches, for is_moving()
  double near_threshold;      // inches, switch point between max-speed and cascaded PID
  double degrees_per_inch;    // conversion factor: rotation sensor degrees per inch of height

  double target_height = 0;
  pros::Task task;
};