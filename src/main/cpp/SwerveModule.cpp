#include "SwerveModule.h"

#include <wpi/numbers>

#include <frc/geometry/Rotation2d.h>


SwerveModule::SwerveModule( const int turnMotorChannel, 
                           const int driveMotorChannel, const int absoluteEncoderChannel, const double absoluteEncoderOffset ) 
                           : m_driveMotor{ driveMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless },
                           m_turnMotor{ turnMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless },
                           m_turnEncoder{ absoluteEncoderChannel, absoluteEncoderOffset } {
    m_drivePIDController.SetP(pidf::kDriveP);
    m_drivePIDController.SetI(pidf::kDriveI);
    m_drivePIDController.SetD(pidf::kDriveD);
    m_drivePIDController.SetFF(pidf::kDriveFF);
    m_turnPIDController.EnableContinuousInput(-180, 180);
}

frc::SwerveModuleState SwerveModule::SetDesiredState( const frc::SwerveModuleState& referenceState ) {
    const auto state = frc::SwerveModuleState::Optimize( referenceState, m_turnEncoder.GetPosition() );

    units::revolutions_per_minute_t rpm = state.speed / physical::kDriveMetersPerRotation;

    m_drivePIDController.SetReference( rpm.value(), rev::CANSparkMaxLowLevel::ControlType::kVelocity );

    m_turnMotor.Set( m_turnPIDController.Calculate( m_turnEncoder.GetPosition().value(), state.angle.Degrees().value() ) );

    return state; 
}