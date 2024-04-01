package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants.NotesConstants;
import frc.robot.LimelightHelpers;

public class NotesVisionSubsystem extends SubsystemBase {
    private NetworkTable LLTable; // unused

    public NotesVisionSubsystem() {
        this.LLTable = NetworkTableInstance.getDefault().getTable(NotesConstants.TABLE_KEY);
    }

    public double aimP(double maxAngularVelocity) {
        return -((LimelightHelpers.getTX(NotesConstants.NAME) * NotesConstants.kP_AIM)
                * maxAngularVelocity);
    }

    public double rangeP(double maxSpeed) {
        return -((LimelightHelpers.getTY(NotesConstants.NAME) * NotesConstants.kP_RANGE) * maxSpeed);
    }
}
