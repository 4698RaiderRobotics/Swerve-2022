// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/XboxController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/ADXRS450_Gyro.h>

#include "SwerveModuleDisplay.h"
#include "SwerveModule.h"
#include "Constants.h"
#include "ControllerAxis.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  SwerveStatusDisplay swerve_display{ "Swerve Drive", "Robot Wheel Status" };

  frc::XboxController m_xbox{0};
  ControllerAxis vx_axis{m_xbox, frc::XboxController::Axis::kLeftY, true};
  ControllerAxis vy_axis{m_xbox, frc::XboxController::Axis::kLeftX, true};
  ControllerAxis omega_axis{m_xbox, frc::XboxController::Axis::kRightX, true};

  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter {3 / 1_s };
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{ 3 / 1_s };
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{ 3 / 1_s };
  
  frc::ChassisSpeeds ch_speeds;

  frc::ADXRS450_Gyro m_gyro{frc::SPI::Port::kOnboardCS0};

  frc::Translation2d m_frontLeftLocation{ +( physical::kDriveBaseWidth / 2 ), +( physical::kDriveBaseWidth / 2 ) };
  frc::Translation2d m_frontRightLocation{ +( physical::kDriveBaseWidth / 2 ), -( physical::kDriveBaseWidth / 2 ) };
  frc::Translation2d m_backLeftLocation{ -( physical::kDriveBaseWidth / 2 ), +( physical::kDriveBaseWidth / 2 ) };
  frc::Translation2d m_backRightLocation{ -( physical::kDriveBaseWidth / 2 ), -( physical::kDriveBaseWidth / 2 ) };

  SwerveModule m_frontLeft{ 1, 2, 0, physical::kFrontLeftAbsoluteOffset };
  SwerveModule m_frontRight{ 7, 8, 1, physical::kFrontRightAbsoluteOffset };
  SwerveModule m_backLeft{ 3, 4, 2, physical::kBackLeftAbsoluteOffset };
  SwerveModule m_backRight{ 5, 6, 3, physical::kBackRightAbsoluteOffset };

  frc::SwerveDriveKinematics<4> m_kinematics{ m_frontLeftLocation, m_frontRightLocation, 
                                              m_backLeftLocation,m_backRightLocation };
};
