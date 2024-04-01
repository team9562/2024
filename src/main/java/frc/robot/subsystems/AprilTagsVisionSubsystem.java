package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants.AprilTagConstants;
import frc.robot.LimelightHelpers;

public class AprilTagsVisionSubsystem extends SubsystemBase {
    private NetworkTable LLTable; // unused

    public AprilTagsVisionSubsystem() {
        this.LLTable = NetworkTableInstance.getDefault().getTable(AprilTagConstants.TABLE_KEY);
    }

    public double aimP(double maxAngularVelocity) {
        return -((LimelightHelpers.getTX(AprilTagConstants.NAME) * AprilTagConstants.kP_AIM)
                * maxAngularVelocity);
    }

    public double rangeP(double maxSpeed) {
        return -((LimelightHelpers.getTY(AprilTagConstants.NAME) * AprilTagConstants.kP_RANGE) * maxSpeed);
    }
}
