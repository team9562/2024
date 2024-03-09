package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class LeaveLine extends SequentialCommandGroup {
    public LeaveLine(SwerveSubsystem swerve) {
        addCommands(
                new RunCommand(() -> swerve.driveFieldOriented(new ChassisSpeeds(-3, 0, 0)), swerve).withTimeout(1),
                new WaitCommand(1),
                new RunCommand(() -> swerve.driveFieldOriented(new ChassisSpeeds(0, 0, 0)), swerve));
    }
}
