package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.types.UpDownDirection;

public class MoveElevator extends Command {
    private final ElevatorSubsystem elevator;

    private final UpDownDirection direction;
    private final double speedPercentage;

    public MoveElevator(ElevatorSubsystem elevator, UpDownDirection direction, double speedPercentage) {
        this.elevator = elevator;
        this.direction = direction;
        this.speedPercentage = speedPercentage;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        switch (direction) {
            default:
            case up:
                if (elevator.getElevatorHeight() < ElevatorConstants.MAX_HEIGHT)
                    elevator.moveElevator(speedPercentage);
                else
                    elevator.stop();
                break;
            case down:
                if (elevator.getElevatorHeight() > ElevatorConstants.MIN_HEIGHT || !elevator.isBottomedOut())
                    elevator.moveElevator(-speedPercentage);
                else
                    elevator.stop();
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
