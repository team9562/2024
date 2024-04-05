package frc.robot.commands.auto;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.AutonConstants.VantagePoints;
import frc.robot.commands.angle.RotateSetpointPercentage;
import frc.robot.commands.shooter.Feed;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.swerve.AutoIntakeCommand;
import frc.robot.commands.swerve.FaceAngleCommand;
import frc.robot.subsystems.AngleSubystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NotesVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.types.AngleSetpoint;
import frc.robot.types.InOutDirection;
import frc.robot.types.SpeakerPosition;

public class FourNoteH213 extends SequentialCommandGroup {
    public FourNoteH213(AngleSubystem angle, ShooterSubsystem shooter, ElevatorSubsystem elevator,
            IntakeSubsystem intake, SwerveSubsystem swerve, NotesVisionSubsystem notesVision) {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

        double driversTargetAngle = isRed ? 0 : 180;
        double driversTargetAngleAway = isRed ? 180 : 0;
        Pose2d shootPoint = isRed ? VantagePoints.PP_SHOOTING_RED : VantagePoints.PP_SHOOTING_BLUE;
        SpeakerPosition speakerPositionBack = isRed ? SpeakerPosition.redMiddleBack : SpeakerPosition.blueMiddleBack;

        addCommands(
                // Preloaded note
                new ParallelDeadlineGroup(
                        new Shoot(shooter, true),
                        new RotateSetpointPercentage(angle, elevator, intake, AngleSetpoint.max, true)
                ),
                new Feed(shooter, InOutDirection.out).withTimeout(ShooterConstants.FEED_DURATION),

                // H2
                swerve.pathfindToSpeaker(speakerPositionBack),
                new ParallelCommandGroup(
                        new FaceAngleCommand(swerve, driversTargetAngleAway),
                        new RotateSetpointPercentage(angle, elevator, intake, AngleSetpoint.min, true)
                ),
                new AutoIntakeCommand(swerve, shooter, intake, angle, notesVision),
                new FaceAngleCommand(swerve, driversTargetAngle),
                new ParallelDeadlineGroup(
                        new Shoot(shooter, true),
                        swerve.pathfind(shootPoint),
                        new RotateSetpointPercentage(angle, elevator, intake, AngleSetpoint.podium, true)
                ),
                new Feed(shooter, InOutDirection.out).withTimeout(ShooterConstants.FEED_DURATION),

                // H1
                new ParallelCommandGroup(
                        new FaceAngleCommand(swerve, driversTargetAngleAway + (isRed ? -90 : 90)),
                        new RotateSetpointPercentage(angle, elevator, intake, AngleSetpoint.min, true)
                ),
                new AutoIntakeCommand(swerve, shooter, intake, angle, notesVision),
                new FaceAngleCommand(swerve, driversTargetAngle),
                new ParallelDeadlineGroup(
                        new Shoot(shooter, true),
                        swerve.pathfind(shootPoint),
                        new RotateSetpointPercentage(angle, elevator, intake, AngleSetpoint.podium, true)
                ),
                new Feed(shooter, InOutDirection.out).withTimeout(ShooterConstants.FEED_DURATION),
                
                // H3
                new FaceAngleCommand(swerve, driversTargetAngleAway + (isRed ? 90 : -90)),
                new ParallelCommandGroup(
                        swerve.pathfind(shootPoint),
                        new RotateSetpointPercentage(angle, elevator, intake, AngleSetpoint.min, true)
                ),
                new AutoIntakeCommand(swerve, shooter, intake, angle, notesVision),
                new FaceAngleCommand(swerve, driversTargetAngle),
                new ParallelDeadlineGroup(
                        new Shoot(shooter, true),
                        swerve.pathfind(shootPoint),
                        new RotateSetpointPercentage(angle, elevator, intake, AngleSetpoint.podium, true)
                ),
                new Feed(shooter, InOutDirection.out).withTimeout(ShooterConstants.FEED_DURATION)
        );

        addRequirements(angle, shooter, elevator, intake, swerve);
    }
}
