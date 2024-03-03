package frc.robot.commands.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.types.InOutDirection;

public class Intake extends Command {
    private final IntakeSubsystem intake;

    private final InOutDirection direction;

    public Intake(IntakeSubsystem intake, InOutDirection direction) {
        this.intake = intake;
        this.direction = direction;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        switch (direction) {
            default:
            case in:
                intake.set(1);
                break;
            case out:
                intake.set(-1);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
