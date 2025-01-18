// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

Robot::Robot() {
  redux::canand::EnsureCANLinkServer();
}
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("Gyro Yaw", ((units::degree_t) gyro.GetYaw()).value());
  frc::SmartDashboard::PutBoolean("Gyro connected", gyro.IsConnected());

  frc::SmartDashboard::PutBoolean("Gyro accelerationSaturation", gyro.GetActiveFaults().accelerationSaturation);
  frc::SmartDashboard::PutBoolean("Gyro angularVelocitySaturation", gyro.GetActiveFaults().angularVelocitySaturation);
  frc::SmartDashboard::PutBoolean("Gyro calibrating", gyro.GetActiveFaults().calibrating);
  frc::SmartDashboard::PutBoolean("Gyro canGeneralError", gyro.GetActiveFaults().canGeneralError);
  frc::SmartDashboard::PutBoolean("Gyro canIdConflict", gyro.GetActiveFaults().canIdConflict);
  frc::SmartDashboard::PutBoolean("Gyro faultsValid", gyro.GetActiveFaults().faultsValid);
  frc::SmartDashboard::PutBoolean("Gyro hardwareFault", gyro.GetActiveFaults().hardwareFault);
  frc::SmartDashboard::PutBoolean("Gyro outOfTemperatureRange", gyro.GetActiveFaults().outOfTemperatureRange);
  frc::SmartDashboard::PutBoolean("Gyro powerCycle", gyro.GetActiveFaults().powerCycle);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
