#pragma once

#include <string>

#include <frc/smartdashboard/SendableChooser.h>

#include <units/velocity.h>

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/controller/PIDController.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>


#include "AbsoluteEncoder.h"
#include "Constants.h"

class SwerveModule {
    public:
        SwerveModule( const int turnMotorChannel, const int driveMotorChannel, const int absoluteEncoderChannel, const double absoluteEncoderOffset );


        // Sets motors to the optimized wheel state and returns the optimized state
        frc::SwerveModuleState SetDesiredState( const frc::SwerveModuleState& state );

    private:
        rev::CANSparkMax m_driveMotor;
        rev::CANSparkMax m_turnMotor;

        rev::SparkMaxRelativeEncoder m_driveEncoder = m_driveMotor.GetEncoder();
        AbsoluteEncoder m_turnEncoder;

        rev::SparkMaxPIDController m_drivePIDController = m_driveMotor.GetPIDController();
        frc2::PIDController m_turnPIDController{ pidf::kTurnP, pidf::kTurnI, pidf::kTurnD };
};