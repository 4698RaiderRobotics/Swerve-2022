// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/MathUtil.h>

#include "Robot.h"
#include "Constants.h"

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  m_gyro.Calibrate();
  m_gyro.Reset();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  ch_speeds.vx = vx_axis.GetAxis() * physical::kMaxDriveSpeed;
  ch_speeds.vy = vy_axis.GetAxis() * physical::kMaxDriveSpeed;
  ch_speeds.omega = omega_axis.GetAxis() * physical::kMaxTurnSpeed;
  
  auto states = m_kinematics.ToSwerveModuleStates( ch_speeds.FromFieldRelativeSpeeds( 
    ch_speeds.vx, ch_speeds.vy, ch_speeds.omega, frc::Rotation2d{ units::degree_t{ -m_gyro.GetAngle() } } ) );
  m_kinematics.DesaturateWheelSpeeds( &states, physical::kMaxDriveSpeed );
  auto [ fl, fr, bl, br ] = states;
  auto flOpp = m_frontLeft.SetDesiredState( fl );
  auto frOpp = m_frontRight.SetDesiredState( fr );
  auto blOpp = m_backLeft.SetDesiredState( bl );
  auto brOpp = m_backRight.SetDesiredState( br );
  wpi::array opStates = {flOpp, frOpp, blOpp, brOpp};
  swerve_display.SetState( opStates );
  frc::SmartDashboard::PutNumber("Gyro", m_gyro.GetAngle());
}

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
