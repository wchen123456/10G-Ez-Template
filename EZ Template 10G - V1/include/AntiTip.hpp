// AntiTip.hpp

#pragma once
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "pros/motors.hpp"

class AntiTip {
 public:
  // imu: your V5 Inertial Sensor
  // drive_motors: your drivetrain motor group (or however your EZ-Template chassis exposes motors)
  // tip_threshold: degrees of forward/backward tilt that triggers lockout (e.g. 35)
  // recovery_threshold: degrees of tilt to return to before releasing lockout (e.g. 15)
  // correction_voltage: how hard to drive in the correction direction (0-12000 mV)
  AntiTip(pros::Imu* imu, pros::MotorGroup* drive_motors,
          double tip_threshold, double recovery_threshold, double correction_voltage = 8000)
      : imu(imu),
        drive_motors(drive_motors),
        tip_threshold(tip_threshold),
        recovery_threshold(recovery_threshold),
        correction_voltage(correction_voltage),
        task([this] { this->task_loop(); }) {}

  // Returns true if anti-tip is currently overriding driver/auton control
  bool is_active() {
    return locked_out;
  }

  // Call this from opcontrol/autonomous BEFORE your normal drive code:
  // if anti-tip is active, skip your normal drive commands this loop
  bool should_override_drive() {
    return locked_out;
  }

 private:
  void task_loop() {
    while (true) {
      double pitch = get_signed_pitch();

      if (!locked_out) {
        // Check if we've tipped past the threshold in either direction
        if (fabs(pitch) > tip_threshold) {
          locked_out = true;
          tip_direction = (pitch > 0) ? 1.0 : -1.0;  // remember which way we tipped
        }
      } else {
        // Currently locked out: drive opposite to the tip direction until recovered
        drive_motors->move_voltage(-tip_direction * correction_voltage);

        if (fabs(pitch) < recovery_threshold) {
          locked_out = false;
          drive_motors->move_voltage(0);  // hand control back cleanly
        }
      }

      pros::delay(10);
    }
  }

  // Converts IMU pitch to a signed forward(+)/backward(-) tilt value
  // NOTE: verify sign convention on your robot -- may need to flip this
  double get_signed_pitch() {
    double pitch = imu->get_pitch();
    // IMU pitch from PROS is typically -180 to 180; adjust wrap-around if needed
    if (pitch > 180) pitch -= 360;
    return pitch;
  }

  pros::Imu* imu;
  pros::MotorGroup* drive_motors;

  double tip_threshold;
  double recovery_threshold;
  double correction_voltage;

  bool locked_out = false;
  double tip_direction = 0;

  pros::Task task;
};