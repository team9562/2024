package frc.robot.commands.auto;

import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.AutonConstants.VantagePoints;
import frc.robot.commands.angle.RotateSetpointPercentage;
import frc.robot.commands.shooter.Feed;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.swerve.AutoIntakeCommand;
import frc.robot.subsystems.AngleSubystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NotesVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.types.AngleSetpoint;
import frc.robot.types.InOutDirection;
import frc.robot.types.SpeakerPosition;

public class TwoNoteC5 extends SequentialCommandGroup {
    public TwoNoteC5(AngleSubystem angle, ShooterSubsystem shooter, ElevatorSubsystem elevator,
            IntakeSubsystem intake, SwerveSubsystem swerve, NotesVisionSubsystem notesVision) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

        addCommands(
                // Preloaded note
                new ParallelDeadlineGroup(
                        new Shoot(shooter, true),
                        new RotateSetpointPercentage(angle, elevator, intake, AngleSetpoint.max, true)
                ),
                new Feed(shooter, InOutDirection.out).withTimeout(ShooterConstants.FEED_DURATION),

                // C5
                new ParallelDeadlineGroup(
                        swerve.pathfind(isRed ? VantagePoints.PP_C5_VANTAGE_POINT_RED : VantagePoints.PP_C5_VANTAGE_POINT_BLUE, 1.25),
                        new RotateSetpointPercentage(angle, elevator, intake, AngleSetpoint.min, true)
                ),
                new AutoIntakeCommand(swerve, shooter, intake, angle, notesVision),
                swerve.pathfindToSpeaker(isRed ? SpeakerPosition.redSourceSide : SpeakerPosition.blueSourceSide),
                new ParallelDeadlineGroup(
                        new Shoot(shooter, true),
                        new RotateSetpointPercentage(angle, elevator, intake, AngleSetpoint.max, true)
                ),
                new Feed(shooter, InOutDirection.out).withTimeout(ShooterConstants.FEED_DURATION)
        );

        addRequirements(angle, shooter, elevator, intake, swerve);
    }
}
