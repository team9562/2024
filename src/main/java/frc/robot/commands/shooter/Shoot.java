package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.types.InOutDirection;

public class Shoot extends Command {
    private final ShooterSubsystem shooter;
    private final boolean auto;

    public Shoot(ShooterSubsystem shooter, boolean auto) {
        this.shooter = shooter;
        this.auto = auto;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setRPMs(MotorConstants.NEO_V1_MAX_RPMS * -InOutDirection.out.percentage);
    }

    @Override
    public boolean isFinished() {
        return auto ? shooter.isFastEnough() : false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooters();
    }
}
