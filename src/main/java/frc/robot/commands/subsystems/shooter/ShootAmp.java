package frc.robot.commands.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAmp extends Command {
    private final ShooterSubsystem shooter;

    public ShootAmp(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.shootAmpMaxSpeed();
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
