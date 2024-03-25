package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AimTowardsNoteCommand extends Command {
    private final SwerveSubsystem swerve;

    public AimTowardsNoteCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.aimTowardsNote();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.lock();
    }
}
