#include "Drivetrain.h"

// Drives with joystick inputs
// This takes -1 to 1 inputs
void Drivetrain::Drive( frc::ChassisSpeeds speeds, bool fieldRelative ) {
    auto states = m_kinematics.ToSwerveModuleStates( fieldRelative ? speeds.FromFieldRelativeSpeeds( 
                    speeds.vx, speeds.vy, speeds.omega, frc::Rotation2d{ units::degree_t{ m_gyro.GetYaw() } } ) :
                    speeds );
    m_kinematics.DesaturateWheelSpeeds( &states, physical::kMaxDriveSpeed );

    auto [ fl, fr, bl, br ] = states;

    auto flState = m_frontLeft.SetDesiredState( fl );
    auto frState = m_frontRight.SetDesiredState( fr );
    auto blState = m_backLeft.SetDesiredState( bl );
    auto brState = m_backRight.SetDesiredState( br );

    wpi::array opStates = {flState, frState, blState, brState};
    swerve_display.SetState( opStates );
}

// Drives a path given a trajectory state
void Drivetrain::DriveTrajectory( frc::Trajectory::State trajectoryState ) {
    

    auto adjustedSpeeds = m_controller.Calculate( m_odometry.GetPose(), trajectoryState, trajectoryState.pose.Rotation().Degrees() );

    auto [fl, fr, bl, br ] = m_kinematics.ToSwerveModuleStates(adjustedSpeeds);

    auto flState = m_frontLeft.SetDesiredState( fl );
    auto frState = m_frontRight.SetDesiredState( fr );
    auto blState = m_backLeft.SetDesiredState( bl );
    auto brState = m_backRight.SetDesiredState( br );

    m_odometry.Update( frc::Rotation2d{ units::degree_t{ m_gyro.GetYaw() } }, flState, frState, blState, brState );

    wpi::array opStates = {flState, frState, blState, brState};
    swerve_display.SetState( opStates );
}

// Returns the pose2d of the robot
frc::Pose2d Drivetrain::GetPose( void ) {
    return m_odometry.GetPose();
}

// Resets the gyro to 0
void Drivetrain::ResetGyro( void ) {
    m_gyro.SetYaw(0);
}

void Drivetrain::ResetPose( void ) {
    m_odometry.ResetPosition( frc::Pose2d{ 0_ft, 0_ft, 0_deg }, frc::Rotation2d{ 0_deg } );
}