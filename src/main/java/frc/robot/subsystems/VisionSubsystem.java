package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// import frc.robot.LimelightHelpers;

public class VisionSubsystem {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    //Pose2D pose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");

    // double limelight_aim_proportional()
    // {
    //     double kP = 0.035;

    //     double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
    //     targetingAngularVelocity *= -1.0;

    //     //return targetingAngularVelocity;


    // }
}
