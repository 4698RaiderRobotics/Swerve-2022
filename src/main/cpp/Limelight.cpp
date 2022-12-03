#include "Limelight.h"

frc::ChassisSpeeds Limelight::TargetRobot( void ) {
    targetX = table->GetNumber("tx",0.0);
    targetY = table->GetNumber("ty",0.0);
    
    t_speeds.vx = -targetY * kxP * physical::kMaxDriveSpeed;
    t_speeds.omega = -targetX * kOmegaP * physical::kMaxTurnSpeed;

    frc::SmartDashboard::PutNumber( "Target X Speed", t_speeds.vx.value() );
    frc::SmartDashboard::PutNumber( "Target Omega Speed", t_speeds.omega.value() );

    return t_speeds;
}

void Limelight::SetPipeline( int pipelineId ) {
    table->PutNumber( "pipeline", pipelineId );
}