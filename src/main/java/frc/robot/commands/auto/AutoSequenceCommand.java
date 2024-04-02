package frc.robot.commands.auto;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.angle.RotateSetpoint;
import frc.robot.commands.shooter.Feed;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.swerve.AutoIntakeCommand;
import frc.robot.commands.swerve.FaceAngleCommand;
import frc.robot.commands.swerve.TurnAroundCommand;
import frc.robot.subsystems.AngleSubystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NotesVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.types.AngleSetpoint;
import frc.robot.types.InOutDirection;
import frc.robot.types.SpeakerPosition;

public class AutoSequenceCommand extends SequentialCommandGroup {
    public AutoSequenceCommand(AngleSubystem angle, ShooterSubsystem shooter, ElevatorSubsystem elevator,
            IntakeSubsystem intake, SwerveSubsystem swerve, NotesVisionSubsystem notesVision) {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        addCommands(
                new ParallelRaceGroup(new RotateSetpoint(angle, elevator, intake, AngleSetpoint.max).withTimeout(1.15),
                        new Shoot(shooter)),
                new Feed(shooter, InOutDirection.out).withTimeout(0.2),

                new ParallelRaceGroup(new TurnAroundCommand(swerve),
                        new RotateSetpoint(angle, elevator, intake, AngleSetpoint.min).withTimeout(1)),
                new AutoIntakeCommand(swerve, shooter, intake, notesVision),
                new FaceAngleCommand(swerve,
                        alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red ? 180 : 0),
                new ParallelRaceGroup(swerve.pathfindToSpeaker(SpeakerPosition.blueMiddle),
                        new RotateSetpoint(angle, elevator, intake, AngleSetpoint.max)),
                new ParallelRaceGroup(
                        new RotateSetpoint(angle, elevator, intake, AngleSetpoint.max)
                                .withTimeout(1.15),
                        new Shoot(shooter)),
                new Feed(shooter, InOutDirection.out).withTimeout(0.2),

                new ParallelRaceGroup(new FaceAngleCommand(swerve, 45),
                        new RotateSetpoint(angle, elevator, intake, AngleSetpoint.min).withTimeout(1)),
                new AutoIntakeCommand(swerve, shooter, intake, notesVision),
                new FaceAngleCommand(swerve,
                        alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red ? 180 : 0),
                new ParallelRaceGroup(swerve.pathfindToSpeaker(SpeakerPosition.blueMiddle),
                        new RotateSetpoint(angle, elevator, intake, AngleSetpoint.max)),
                new ParallelRaceGroup(
                        new RotateSetpoint(angle, elevator, intake, AngleSetpoint.max)
                                .withTimeout(1.15),
                        new Shoot(shooter)),
                new Feed(shooter, InOutDirection.out).withTimeout(0.2),

                new ParallelRaceGroup(new FaceAngleCommand(swerve, 315),
                        new RotateSetpoint(angle, elevator, intake, AngleSetpoint.min).withTimeout(1)),
                new AutoIntakeCommand(swerve, shooter, intake, notesVision),
                new FaceAngleCommand(swerve,
                        alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red ? 180 : 0),
                new ParallelRaceGroup(swerve.pathfindToSpeaker(SpeakerPosition.blueMiddle),
                        new RotateSetpoint(angle, elevator, intake, AngleSetpoint.max)),
                new ParallelRaceGroup(
                        new RotateSetpoint(angle, elevator, intake, AngleSetpoint.max)
                                .withTimeout(1.15),
                        new Shoot(shooter)),
                new Feed(shooter, InOutDirection.out).withTimeout(0.2));

        addRequirements(angle, shooter, elevator, intake, swerve);
    }
}
