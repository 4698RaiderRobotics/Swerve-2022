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
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/ADXRS450_Gyro.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>
#include <units/time.h>
#include <frc/smartdashboard/Field2d.h>

#include "SwerveModuleDisplay.h"
#include "SwerveModule.h"
#include "Constants.h"
#include "ControllerAxis.h"
#include "Drivetrain.h"
#include "Limelight.h"

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

  frc::XboxController m_xbox{0};
  ControllerAxis vx_axis{m_xbox, frc::XboxController::Axis::kLeftY, true};
  ControllerAxis vy_axis{m_xbox, frc::XboxController::Axis::kLeftX, true};
  ControllerAxis omega_axis{m_xbox, frc::XboxController::Axis::kRightX, true};

  Drivetrain m_drivetrain;
  frc::ChassisSpeeds m_speeds; 

  Limelight m_limelight;
  
  frc::Field2d m_field;

  frc::Trajectory m_trajectory;

  units::second_t m_autoElapsed;
};