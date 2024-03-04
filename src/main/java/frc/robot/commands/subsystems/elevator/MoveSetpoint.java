package frc.robot.commands.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.types.ElevatorSetpoint;

public class MoveSetpoint extends Command {
    private final ElevatorSubsystem elevator;

    private final ElevatorSetpoint setpoint;

    public MoveSetpoint(ElevatorSubsystem elevator, ElevatorSetpoint setpoint) {
        this.elevator = elevator;
        this.setpoint = setpoint;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        switch (setpoint) {
            default:
            case max:
                elevator.setElevatorPosition(1);
                break;
            case min:    
                elevator.setElevatorPosition(0);
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
