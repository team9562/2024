package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.subsystems.angle.RotateSetpoint;
import frc.robot.commands.subsystems.shooter.Feed;
import frc.robot.commands.subsystems.shooter.Shoot;
import frc.robot.subsystems.AngleSubystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.types.AngleSetpoint;
import frc.robot.types.InOutDirection;

public class SpeakerStart extends SequentialCommandGroup {
    public SpeakerStart(SwerveSubsystem swerve, AngleSubystem angle, ShooterSubsystem shooter,
            ElevatorSubsystem elevator, IntakeSubsystem intake) {
        addCommands(
                new RotateSetpoint(angle, elevator, intake, AngleSetpoint.max).withTimeout(1),
                new WaitCommand(1),
                new Shoot(shooter, InOutDirection.out).withTimeout(1.75),
                new WaitCommand(1),
                new Feed(shooter, InOutDirection.out).withTimeout(0.5),
                new WaitCommand(0.5),
                new RunCommand(() -> swerve.driveFieldOriented(new ChassisSpeeds(3, 0, 0)), swerve).withTimeout(0.5),
                new WaitCommand(0.5),
                new RunCommand(() -> swerve.driveFieldOriented(new ChassisSpeeds(0, 0, 0)), swerve));
    }
}
