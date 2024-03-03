package frc.robot.commands.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.types.InOutDirection;

public class Shoot extends Command {
    private final ShooterSubsystem shooter;

    private final InOutDirection direction;

    public Shoot(ShooterSubsystem shooter, InOutDirection direction) {
        this.shooter = shooter;
        this.direction = direction;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        switch (direction) {
            default:
            case out:
                shooter.setRPMs(MotorConstants.NEO_V1_MAX_RPMS);
                break;
            case in:
                shooter.setRPMs(-MotorConstants.NEO_V1_MAX_RPMS);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooters();
    }
}
