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

// Sets each individual SwerveModule to a given SwerveModuleState
frc::SwerveModuleState SwerveModule::SetDesiredState( const frc::SwerveModuleState& referenceState ) {
    const auto state = frc::SwerveModuleState::Optimize( referenceState, m_turnEncoder.GetPosition() );

    units::revolutions_per_minute_t rpm = state.speed / physical::kDriveMetersPerRotation;

    // The onboard PID controller has a SetReference() function that automatically sets the motor to the correct speed.
    m_drivePIDController.SetReference( rpm.value(), rev::CANSparkMaxLowLevel::ControlType::kVelocity );

    // The software PID controller outputs a value 0 to 1 which must be set using the Set() function of the motor.
    m_turnMotor.Set( m_turnPIDController.Calculate( m_turnEncoder.GetPosition().value(), state.angle.Degrees().value() ) );

    return state; 
}

// Outputs the current state of the SwerveModule
frc::SwerveModuleState SwerveModule::GetState( void ) {
    return { units::meters_per_second_t{ m_driveEncoder.GetVelocity() }, units::radian_t{ m_turnEncoder.GetPosition() } };
}