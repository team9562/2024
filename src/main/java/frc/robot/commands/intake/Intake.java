package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AngleSubystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.types.InOutDirection;

public class Intake extends Command {
    private final IntakeSubsystem intake;
    private final AngleSubystem angle;

    private final InOutDirection direction;

    public Intake(IntakeSubsystem intake, AngleSubystem angle, InOutDirection direction) {
        this.intake = intake;
        this.angle = angle;
        this.direction = direction;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.set(direction.percentage);
        // if (direction == InOutDirection.in && angle.isAtMin()) {
        // } else intake.set(direction.percentage);
    }

    @Override
    public boolean isFinished() {
        if (direction == InOutDirection.in) return !angle.isAtMin();
        else return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
