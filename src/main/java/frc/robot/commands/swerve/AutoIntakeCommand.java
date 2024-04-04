package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.AngleSubystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NotesVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.types.InOutDirection;

public class AutoIntakeCommand extends Command {
    private final SwerveSubsystem swerve;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final AngleSubystem angle;
    private final NotesVisionSubsystem notesVision;

    public AutoIntakeCommand(SwerveSubsystem swerve, ShooterSubsystem shooter, IntakeSubsystem intake,
            AngleSubystem angle, NotesVisionSubsystem notesVision) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.intake = intake;
        this.angle = angle;
        this.notesVision = notesVision;

        addRequirements(swerve, shooter, intake);
    }

    @Override
    public void execute() {
        intake.set(InOutDirection.in.percentage);

        shooter.setRPMs(MotorConstants.NEO_V1_MAX_RPMS * -InOutDirection.in.percentage);
        shooter.setFeeder(MotorConstants.NEO_550_MAX_RPMS * InOutDirection.in.percentage);

        swerve.driveLLAim(() -> Math.abs(notesVision.rangeP(swerve.maximumSpeed)),
                () -> notesVision.aimP(swerve.getSwerveController().config.maxAngularVelocity));
    }

    @Override
    public boolean isFinished() {
        return shooter.isBottomedOut() || !angle.isAtMin();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.lock();
        shooter.stopAll();
        intake.stop();
    }
}
