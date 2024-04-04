package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AngleConstants;
import frc.robot.Constants.VisionConstants.AprilTagConstants;

public class AprilTagsVisionSubsystem extends SubsystemBase {
    private NetworkTable LLTable; // unused

    public AprilTagsVisionSubsystem() {
        this.LLTable = NetworkTableInstance.getDefault().getTable(AprilTagConstants.TABLE_KEY);
    }

    public double calculateShooterAngle() {
        double angle = Units.radiansToDegrees(Math.tan((AprilTagConstants.SPEAKER_HEIGHT - AprilTagConstants.APRILTAG_HEIGHT) / estimateDistance())) - AngleConstants.ANGLE_OFFSET_FLAT;
        System.out.println(angle);
        return angle;
    }

    public double estimateDistance() {
        double angleToAprilTag = Units.degreesToRadians((90 - AprilTagConstants.LIMELIGHT_ANGLE) + LLTable.getEntry("ty").getDouble(0));
        return (AprilTagConstants.APRILTAG_HEIGHT - AprilTagConstants.LIMELIGHT_HEIGHT) / Math.tan(angleToAprilTag);
    }
}
