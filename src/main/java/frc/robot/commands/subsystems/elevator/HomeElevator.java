package frc.robot.commands.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AngleSubystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class HomeElevator extends Command {
    private final ElevatorSubsystem elevator;
    private final AngleSubystem angle;

    public HomeElevator(ElevatorSubsystem elevator, AngleSubystem angle) {
        this.elevator = elevator;
        this.angle = angle;

        // addRequirements(elevator, angle);
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.stop();

        // angle.moveFlush();
    }

    @Override
    public void execute() {
        // if (!elevator.isBottomedOut() && angle.isAtTargetAngle())
        if (!elevator.isBottomedOut())
            elevator.moveElevator(-0.05);
    }

    @Override
    public boolean isFinished() {
        return elevator.isBottomedOut();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted)
            elevator.resetElevatorEncoder();

        elevator.stop();
    }
}
