// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/MathUtil.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include "Robot.h"


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  frc::SmartDashboard::PutData( "Field", &m_field );

  m_drivetrain.ResetGyro();
  m_drivetrain.ResetPose();
}


void Robot::RobotPeriodic() {
  m_field.SetRobotPose( m_drivetrain.GetPose() );
}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();

  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    const frc::Pose2d startPose{ 0_ft, 0_ft, frc::Rotation2d{ 0_deg } };
    const frc::Pose2d endPose{ 8_ft, 8_ft, frc::Rotation2d{ 90_deg } };

    std::vector<frc::Translation2d> interiorWaypoints{
      frc::Translation2d{ 5.65_ft, 2.35_ft }
    };

    frc::TrajectoryConfig config{ 7_fps, 5_fps_sq };

    m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(startPose, interiorWaypoints, endPose, config);
  } else {
    const frc::Pose2d startPose{ 0_ft, 0_ft, frc::Rotation2d{ 0_deg } };
    const frc::Pose2d endPose{ 8_ft, 0_ft, frc::Rotation2d{ 0_deg } };

    std::vector<frc::Translation2d> interiorWaypoints{
      frc::Translation2d{ 4_ft, 0_ft }
    };

    frc::TrajectoryConfig config{ 7_fps, 5_fps_sq };

    m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(startPose, interiorWaypoints, endPose, config);
  }

  m_field.GetObject("traj")->SetTrajectory(m_trajectory);

  m_autoElapsed = 0_ms;
}

void Robot::AutonomousPeriodic() {
  auto goal = m_trajectory.Sample( m_autoElapsed );

  m_drivetrain.DriveTrajectory( goal );

  m_autoElapsed += 20_ms;
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  m_drivetrain.DriveWithJoystick( vx_axis.GetAxis(), vy_axis.GetAxis(), omega_axis.GetAxis() );
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
