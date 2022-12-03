#pragma once

#include <frc/kinematics/ChassisSpeeds.h>

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"

#include "Constants.h"

class Limelight {
    public:
        frc::ChassisSpeeds TargetRobot( void );

        void SetPipeline( int pipelineId );

    private:
        std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

        // The X position of the target in the limelight's view
        double targetX;
        // The Y position of the target in the limelight's view
        double targetY;

        double kOmegaP = 0.02;
        double kxP = 0.05;

        frc::ChassisSpeeds t_speeds;
};