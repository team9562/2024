package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.angle.RotateSetpointPercentage;
import frc.robot.commands.shooter.Feed;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.AngleSubystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.types.AngleSetpoint;
import frc.robot.types.InOutDirection;

public class MessUpCenterNotesCommand extends SequentialCommandGroup {
    public MessUpCenterNotesCommand(AngleSubystem angle, ShooterSubsystem shooter, ElevatorSubsystem elevator,
            IntakeSubsystem intake, SwerveSubsystem swerve, BooleanSupplier isSourceSide) {
        double feedDelay = 0.125;

        addCommands(
                // Preloaded note
                new ParallelDeadlineGroup(
                        new Shoot(shooter, true),
                        new RotateSetpointPercentage(angle, elevator, intake, AngleSetpoint.max, true)
                ),
                new Feed(shooter, InOutDirection.out).withTimeout(feedDelay),

                // Mess up notes
                swerve.pathFindThenPath(PathPlannerPath.fromPathFile(isSourceSide.getAsBoolean() ? "C1 - C5" : "C5 - C1"))
        );

        addRequirements(angle, shooter, elevator, intake, swerve);
    }
}
