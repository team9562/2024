package frc.robot.commands.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAlign extends SequentialCommandGroup {
    public AutoAlign(VisionSubsystem vision, SwerveSubsystem swerve) {
        swerve.resetOdometry(vision.getPose2d());
        addCommands(vision.autoAlignFollowPath());
    }
}
