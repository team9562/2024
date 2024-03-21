package frc.robot.commands.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.types.InOutDirection;

public class ShooterIntake extends Command {
    private final ShooterSubsystem shooter;

    public ShooterIntake(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setRPMs(MotorConstants.NEO_V1_MAX_RPMS * -InOutDirection.in.percentage);
        shooter.setFeeder(MotorConstants.NEO_550_MAX_RPMS * InOutDirection.in.percentage);
    }

    @Override
    public boolean isFinished() {
        return shooter.isBottomedOut();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();
    }
}
